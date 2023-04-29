int _read_thread(lvm_dev_t *dev)
{
	//check input
	if (NULL == dev)
	{
		lvm_put_event(NULL, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"rd_thread input dev error!\r\n");
		return -1;
	}

	lvm_buf_t *buf = dev->buffer;
	buf_manage_t *bm = (buf_manage_t*)dev->buffer->reserved;

	//check buffer properties
	if (buf == NULL || bm == NULL)
	{
		lvm_put_event(NULL, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"rd_thread dev buffer error!\r\n");
		return -1;
	}

	lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"rd_thread run\n");

	//和middle ware共用的锁，现在可以用bm这块内存了
	std::unique_lock<std::mutex> wr_locker(bm->mu_write);

	// sync mode user space handshake,现在还没有锁 bm, 但是bm的初始化已经完成了
	std::unique_lock<std::mutex> rt_ready_locker(bm->mu_rt_ready, std::defer_lock);

	/*
	>>> high speed mode:
	if r_lnum<w_lnum, read MIN(lines in current frame, lines difference)
	one read limited within one buffer frame
	if r_off reach buffer frame height, r_id++, r_fnum++, r_off reset;
	if EOF marked, r_id++, r_fnum++, r_off reset;
	*/
	if (bm->hs_mode)
	{
		int frame_jump = 0;
		void * head_cp = malloc(buf->head_size);
		if(head_cp == NULL)
		{
			lvm_put_event(dev, DBG_INFO_EN, LVM_ERROR_EVENT, DBG_INFO_CODE, "head_cp malloc failed\n");
		}
		/*
		EOF condition logic:
				if it's EOF, send if any, goto next one
				if it's not eof, send if any
				if reach frame end, goto next one
		*/
		while (bm->read_thread_run)
		{
			//printf("_read_thread----bm->r_lnum = %d, bm->w_lnum = %d, bm->r_fnum = %d, bm->w_fnum = %d\n", bm->r_lnum, bm->w_lnum, bm->r_fnum, bm->w_fnum);
			while ((bm->r_lnum != bm->w_lnum)||(bm->r_fnum != bm->w_fnum))
			{
				if (0 == bm->read_thread_run)
				{
					wr_locker.unlock();
					return 0;
				}

				frame_jump = 0;

				//获取当前要读的行数 = 总共写入的 减去 已经读取的行数
				uint32_t ldiff = bm->w_lnum - bm->r_lnum;//cong: 怎么写的?

				//获取当前的帧
				lvm_frame_head_t *frame = (lvm_frame_head_t*)buf_frame_at(buf, bm->r_id);

				//获取当前要读的剩余行数
				uint32_t current_remain = frame->height - bm->r_off;

				int lines = min(ldiff, current_remain);

				if ((frame->flags & FRAME_EOF) || (bm->r_off + lines == buf->height))
				{
					frame_jump = 1;
				}
				memcpy(head_cp, frame, buf->head_size);

                /* in high speed mode, the small frames not at the beginning should not have SOF set
                    prevent SOF storm confusing users */
                if (bm->r_off)
                {
                    ((lvm_frame_head_t*)head_cp)->flags &= ~FRAME_SOF;
                }

                //lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,
                //    "bm->r_lnum = %d; bm->w_lnum = %d; bm->r_fnum = %d; bm->w_fnum = %d;\n", bm->r_lnum,
                //    bm->w_lnum, bm->r_fnum, bm->w_fnum);
				//wr_locker.unlock();
                if (dev->frame_cb)
                {
					if (1 == ((middleware_t *)dev->hw)->stop_flag)
					{
						wr_locker.unlock();
						return -1;
					}
                    //middleware_t *mw = (middleware_t*)dev->hw;
                    //if (mw->debug_mode)
                    //{
                    //    if (dev->buffer->type == LVM_BT_IMAGE)
                    //        frame_cb_img(dev, head_cp, buf_frame_line_at(buf, frame, bm->r_off), lines);
                    //    else if (dev->buffer->type == LVM_BT_POINT_CLOUD)
                    //        frame_cb_pcld(dev, head_cp, buf_frame_line_at(buf, frame, bm->r_off), lines);
                    //    else if (dev->buffer->type == LVM_BT_DEPTH_MAP)
                    //        frame_cb_depth_map(dev, head_cp, buf_frame_line_at(buf, frame, bm->r_off), lines);
                    //}

					//data_mode =  1:深度图 2：强度图 3：深度图 & 强度图  4 : 轮廓仪
					if (dev->capture_param->capture_data_type == LVM_BT_DEPTH_MAP 
						&& ((middleware_t *)dev->hw)->net_dev->working_para.pcld.data_mode != 2) 
					{
						lvm_depth_map_t *dm = (lvm_depth_map_t*)head_cp;
						//alg_proc_hs_data(dev, bm->uniformity_enable, bm->intensity_enable,
						//	head_cp, buf_frame_line_at(buf, frame, bm->r_off), 0, 0, lines);
						//深度图高速（分段采集）模式下（非高速模式和同步采集模式均会做同样处理，数据源会变成整个缓存帧），对每一段内存数据进行调用算法XZ补偿，先把深度图转换成点云，对点云补偿，再转回深度图，
						//其中8个补偿参数，5个compen，3个laser,还需要传入平移参数T[3],另外有5个转换compen系数和原来的compen求和，默认全为0
						if (1 == ((middleware_t *)dev->hw)->xz_compen_enable)
						{
							lvm_point_t *point = (lvm_point_t*)malloc(dm->head.width*lines * sizeof(lvm_point_t));
							dm_data_to_point(dm, lines, buf_frame_line_at(buf, frame, bm->r_off), point);
							vector<double> compen_param; compen_param.clear();
							vector<double> laser_param; laser_param.clear();
							vector<double> transf_param; transf_param.clear();
							for (int i = 0; i < 5; i++)
							{
								compen_param.push_back(((middleware_t *)dev->hw)->net_dev->device_para.part2.Compen[i] +
									((middleware_t *)dev->hw)->net_dev->working_para.peripheral.laser.Compen[i]);
							}
							for (int i = 0; i < 3; i++)
							{
								laser_param.push_back(((middleware_t *)dev->hw)->net_dev->device_para.part2.Laser[i]);
								transf_param.push_back(((middleware_t *)dev->hw)->net_dev->device_para.part2.T[i]);
							}
							compensatehsdataPoints(point, dm, buf_frame_line_at(buf, frame, bm->r_off), dm->head.width, lines, laser_param, transf_param, compen_param);
							free(point);
							point = NULL;
						}
						//深度图高速（分段采集）模式下（非高速模式和同步采集模式均会做同样处理，数据源会变成整个缓存帧），
						//对每一段内存数据进行补缺算法或中值滤波（中位数）或均值滤波，对深度图直接处理，传入滤波距离，此模式下以下三个算法Y方向无效
						//滤波距离表示进行滤波时的小方格的长宽，输入的宽度/点距（x_scale或者y_scale）
						
						if (dm->param->x_scale != 0 && dm->param->y_scale != 0)
						{
							lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE, "377 filter\n");
							if (!median_filter(buf_frame_line_at(buf, frame, bm->r_off), dm->head.width, lines,
								dev->config_param->median_x / dm->param->x_scale, 0,
								((middleware_t *)dev->hw)->stop_flag))
							{
								lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"median filter invalid or failed\n");
							}
							if (!average_filter(buf_frame_line_at(buf, frame, bm->r_off), dm->head.width, lines,
								dev->config_param->average_x / dm->param->x_scale, 0,
								((middleware_t *)dev->hw)->stop_flag))
							{
								lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"average filter invalid or failed\n");
							}
							if (!null_filler_filter(buf_frame_line_at(buf, frame, bm->r_off), dm->head.width, lines,
								dev->config_param->null_filler_x / dm->param->x_scale, 0,
								((middleware_t *)dev->hw)->stop_flag))
							{
								lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"null filter invalid or failed\n");
							}
						}

						//深度图高速（分段采集）模式下（非高速模式和同步采集模式均会做同样处理，数据源会变成整个缓存帧），对每一段内存数据进行倾斜补正，需要将深度图转换成点云后进行补正，之后再将数据转回原来的深度图中
						//参数用到calib_param中的world_rotation和world_translation,这里是经过Capture标定的参数，使用时需要调用lvm_enable_corr_angle接口，enable设置为1
						//SDK会将calib_param中的参数备份到world_rotation_backup和world_translation_backup中，把calib_param中的参数重新设置为单位矩阵
						//一般这里金在XZ补偿和倾斜补正同时使用的场景下用，保证采集的原始数据是没有经过倾斜补正的数据进行XZ补偿
						//使用完毕再用lvm_enable_corr_angle接口，enable设置为0，设置回原来标定好的参数，保证Captrue正常使用
						if (1 == ((middleware_t *)dev->hw)->corr_enable)
						{
							lvm_point_t *point = (lvm_point_t*)malloc(dm->head.width*lines * sizeof(lvm_point_t));
							dm_data_to_point(dm, lines, buf_frame_line_at(buf, frame, bm->r_off), point);
							vector<float> rw; rw.clear();
							vector<float> tw; tw.clear();
							float x_min = 0.0f, x_max = 0.0f;
							for (int i = 0; i < 9; i++)
							{
								rw.push_back(((middleware_t *)dev->hw)->world_rotation_backup[i]);
							}
							for (int i = 0; i < 3; i++)
							{
								tw.push_back(((middleware_t *)dev->hw)->world_translation_backup[i]);
							}
							angleCorrection(point, dm->head.width, lines, rw, tw, x_min, x_max);
							if (x_min >= dm->param->x_min && x_max <= dm->param->x_max)
							{
								void *data_t = buf_frame_line_at(buf, frame, bm->r_off);
								point_to_dm_data(point, data_t, dm->head.width, lines, dm->param->z_scale);
							}
							else
							{
								lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE, "x_min&x_max range error!!!!\n");
							}
							free(point);
							point = NULL;
						}
						((middleware_t *)dev->hw)->md.x_offset = dev->depth_map_param->x_scale*dm->x_offset;
						((middleware_t *)dev->hw)->md.y_offset = dev->depth_map_param->y_scale*dm->y_offset;
					}
					else if (dev->capture_param->capture_data_type == LVM_BT_POINT_CLOUD)
					{

						lvm_point_cloud_t *pcld = (lvm_point_cloud_t*)head_cp;
						/*alg_proc_hs_data(dev, bm->uniformity_enable, bm->intensity_enable,
							head_cp, pcld_dm_buf_frame_line_at(buf, frame, pcld->dm->head.width * sizeof(ushort), bm->r_off), 
							pcld_dm_buf_frame_line_at(buf, frame, pcld->dm->head.width * sizeof(ushort), bm->r_off),
							pcld_intensity_img_buf_frame_line_at(buf, frame, pcld->insensity_img->head.width * sizeof(ushort), bm->r_off),
							lines);*/
						lvm_point_t *point = (lvm_point_t*)buf_frame_line_at(buf, frame, bm->r_off);


						//XZ补偿
						//点云高速（分段采集）模式下（非高速模式和同步采集模式均会做同样处理，数据源会变成整个缓存帧），对每一段内存数据进行调用算法XZ补偿，对点云直接补偿，
						//其中8个补偿参数，5个compen，3个laser,还需要传入平移参数T[3],另外有5个转换compen系数和原来的compen求和，默认全为0
						if (1 == ((middleware_t *)dev->hw)->xz_compen_enable)
						{
							vector<double> compen_param; compen_param.clear();
							vector<double> laser_param; laser_param.clear();
							vector<double> transf_param; transf_param.clear();
							for (int i = 0; i < 5; i++)
							{
								compen_param.push_back(((middleware_t *)dev->hw)->net_dev->device_para.part2.Compen[i] +
									((middleware_t *)dev->hw)->net_dev->working_para.peripheral.laser.Compen[i]);
								//lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"****************compen_param[%d] = %.6f\n", i, compen_param[i]);
							}
							for (int i = 0; i < 3; i++)
							{
								laser_param.push_back(((middleware_t *)dev->hw)->net_dev->device_para.part2.Laser[i]);
								//lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"***************laser_param[%d] = %.6f\n", i, laser_param[i]);
								transf_param.push_back(((middleware_t *)dev->hw)->net_dev->device_para.part2.T[i]);
								//lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"***************transf_param[%d] = %.6f\n", i, transf_param[i]);
							}
							lvm_depth_map_t *dm = NULL;
							bool success = compensatePoints(point, dm, pcld->head.width, lines, laser_param, transf_param, compen_param);
							if (!success)
							{
								lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE, "compensatePoints failed\n");
							}
						}
						//这里是否将点云转换为深度图（非高速模式和同步采集模式均会做同样处理，数据源会变成整个缓存帧），
						//转成深度图后会将其认为是和上面深度图模式一样的方式进行处理
						if (bm->uniformity_enable)
						{
							cv::Mat dst, intensity_dst;
							dst.release();
							intensity_dst.release();
						

							dst = cv::Mat(lines, pcld->dm->head.width, CV_16UC1, pcld_dm_buf_frame_line_at(buf, frame, pcld->dm->head.width * sizeof(ushort), bm->r_off));
							memset(dst.data, 0, dst.cols*dst.rows * sizeof(ushort));
							pcld->dm->head.height = lines;
							if (pcld_to_depth_map(point, pcld->head.width, lines, dst, *pcld->dm->param))
								pcld->dm->data = pcld_dm_buf_frame_line_at(buf, frame, pcld->dm->head.width * sizeof(ushort), bm->r_off);

							if (bm->intensity_enable)
							{
								//get intensity img
								intensity_dst = cv::Mat(lines, pcld->insensity_img->head.width, CV_16UC1, pcld_intensity_img_buf_frame_line_at(buf, frame, pcld->insensity_img->head.width * sizeof(ushort), bm->r_off));
								memset(intensity_dst.data, 0, intensity_dst.cols*intensity_dst.rows * sizeof(ushort));
								pcld->insensity_img->head.height = lines;
								if (pcld_to_intensity_map(point, pcld->head.width, lines, intensity_dst, *pcld->dm->param))
									pcld->insensity_img->data = pcld_intensity_img_buf_frame_line_at(buf, frame, pcld->insensity_img->head.width * sizeof(ushort), bm->r_off);
								
							}

							if (pcld->dm->param->x_scale != 0 && pcld->dm->param->y_scale != 0)
							{
								if (!median_filter(pcld->dm->data, pcld->dm->head.width, pcld->dm->head.height, 
									dev->config_param->median_x / pcld->dm->param->x_scale, 0,
									((middleware_t *)dev->hw)->stop_flag))
								{
									lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"median filter invalid or failed\n");
								}
								if (!average_filter(pcld->dm->data, pcld->dm->head.width, pcld->dm->head.height, 
									dev->config_param->average_x / pcld->dm->param->x_scale, 0,
									((middleware_t *)dev->hw)->stop_flag))
								{
									lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"average filter invalid or failed\n");
								}
								if (!null_filler_filter(pcld->dm->data, pcld->dm->head.width, pcld->dm->head.height,
									dev->config_param->null_filler_x / pcld->dm->param->x_scale, 0,
									((middleware_t *)dev->hw)->stop_flag))
								{
									lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"null filter invalid or failed\n");
								}
							}

							if (1 == ((middleware_t *)dev->hw)->corr_enable)
							{

								vector<float> rw; rw.clear();
								vector<float> tw; tw.clear();
								float x_min = 0.0f, x_max = 0.0f;
								for (int i = 0; i < 9; i++)
								{
									rw.push_back(((middleware_t *)dev->hw)->world_rotation_backup[i]);
								}
								for (int i = 0; i < 3; i++)
								{
									tw.push_back(((middleware_t *)dev->hw)->world_translation_backup[i]);
								}
								angleCorrection(point, pcld->dm->head.width, pcld->dm->head.height, rw, tw, x_min, x_max);
								if (x_min >= pcld->dm->param->x_min && x_max <= pcld->dm->param->x_max)
								{
									void *data_t = pcld->dm->data;
									point_to_dm_data(point, data_t, pcld->dm->head.width, lines, pcld->dm->param->z_scale);
								}
								else
								{
									lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE, "x_min&x_max range error!!!!\n");
								}
							}
							((middleware_t *)dev->hw)->md.x_offset = dev->depth_map_param->x_scale*pcld->dm->x_offset;
							((middleware_t *)dev->hw)->md.y_offset = dev->depth_map_param->y_scale*pcld->dm->y_offset;
							if (((middleware_t *)dev->hw)->plugin_enable)
							{
								lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE, "use plugin!!!!\n");
								list<plugin_obj> objs = get_dev_current_tasks(dev->dev_info->sn);
								if (objs.size() != 0)
								{
									data_frame * df = new data_frame(((middleware_t *)dev->hw)->md.fid, DF_DEPTH, pcld->dm->head.width, lines,
										((middleware_t *)dev->hw)->md, pcld->dm->data, 0);

									df->axis_calib.dir[0] = dev->calib_param->dir[0];
									df->axis_calib.dir[1] = dev->calib_param->dir[1];
									df->axis_calib.dir[2] = dev->calib_param->dir[2];

									df->metadata.x_min = dev->depth_map_param->x_min;
									df->metadata.x_max = dev->depth_map_param->x_max;
									df->metadata.interpolation_len_max = dev->depth_map_param->interpolation_len_max;

									processAndChangeDataFrameWithPlugin(dev->dev_info->sn, df);

									pcld->dm->head.width = df->w;
									pcld->dm->head.height = df->h;
									pcld->dm->x_offset = df->metadata.x_offset;
									pcld->dm->y_offset = df->metadata.y_offset;
									pcld->dm->param->x_min = df->metadata.x_min;
									pcld->dm->param->x_max = df->metadata.x_max;

									memcpy(pcld->dm->data, df->data, df->w*df->h * 2);

									delete df;
								}
							}

							//单独数据包处理完毕后，扔给上层时使用总的（深度图）
							pcld->dm->head.height += bm->r_off;
							pcld->dm->data = pcld_dm_buf_frame_line_at(buf, frame, pcld->dm->head.width * sizeof(ushort), 0);
							if (bm->intensity_enable)
							{
								pcld->insensity_img->head.height += bm->r_off;
								pcld->insensity_img->data = pcld_intensity_img_buf_frame_line_at(buf, frame, pcld->insensity_img->head.width * sizeof(ushort), 0);
							}
						}//uniformity_enable
						else//非均匀点距
						{
							//点云高速（分段采集）模式下（非高速模式和同步采集模式均会做同样处理，数据源会变成整个缓存帧），对每一段内存数据进行倾斜补正
							//参数用到calib_param中的world_rotation和world_translation,这里是经过Capture标定的参数，使用时需要调用lvm_enable_corr_angle接口，enable设置为1
							//SDK会将calib_param中的参数备份到world_rotation_backup和world_translation_backup中，把calib_param中的参数重新设置为单位矩阵
							//一般这里金在XZ补偿和倾斜补正同时使用的场景下用，保证采集的原始数据是没有经过倾斜补正的数据进行XZ补偿
							//使用完毕再用lvm_enable_corr_angle接口，enable设置为0，设置回原来标定好的参数，保证Captrue正常使用
							if (1 == ((middleware_t *)dev->hw)->corr_enable)
							{

								vector<float> rw; rw.clear();
								vector<float> tw; tw.clear();
								float x_min = 0.0f, x_max = 0.0f;
								for (int i = 0; i < 9; i++)
								{
									rw.push_back(((middleware_t *)dev->hw)->world_rotation_backup[i]);
								}
								for (int i = 0; i < 3; i++)
								{
									tw.push_back(((middleware_t *)dev->hw)->world_translation_backup[i]);
								}
								angleCorrection(point, pcld->head.width, pcld->head.height, rw, tw, x_min, x_max);
							}

						}
					}

					if (1 == ((middleware_t *)dev->hw)->stop_flag)
					{
						wr_locker.unlock();
						return -1;
					}

					//填充metadata信息
					time_t cur_time = time(0);
					struct tm tm;
					tm = *localtime(&cur_time);
					strftime(((middleware_t *)dev->hw)->md.time, sizeof(((middleware_t *)dev->hw)->md.time), "%Y:%m:%d %H:%M:%S", &tm);
					//strcpy(p->md.time, asctime(localtime(&cur_time)));
					((middleware_t *)dev->hw)->md.points_of_line = frame->width;
					((middleware_t *)dev->hw)->md.grab_lines = lines;
					((middleware_t *)dev->hw)->md.x_scale = dev->depth_map_param->x_scale;
					((middleware_t *)dev->hw)->md.y_scale = dev->depth_map_param->y_scale;
					((middleware_t *)dev->hw)->md.z_scale = dev->depth_map_param->z_scale;
					((middleware_t *)dev->hw)->md.z_offset = dev->depth_map_param->z_min;
					((middleware_t *)dev->hw)->md.fid = frame->fid;
					((middleware_t *)dev->hw)->md.sid = frame->sid;
					((middleware_t *)dev->hw)->md.time_stamp = frame->time_stamp;
					((middleware_t *)dev->hw)->md.lost_lines = frame->lost_lines;
					((middleware_t *)dev->hw)->md.trigger_min_interval = frame->trigger_min_interval;
					((middleware_t *)dev->hw)->md.trigger_max_interval = frame->trigger_max_interval;
					((middleware_t *)dev->hw)->md.flags = frame->flags;
					((middleware_t *)dev->hw)->md.temperature_j28 = dev->status->temperature_j28;
					((middleware_t *)dev->hw)->md.temperature_j29 = dev->status->temperature_j29;
					((middleware_t *)dev->hw)->md.temperature_j30 = dev->status->temperature_j30;
					lvm_buf_type_t freqType = LVM_BT_IMAGE;
					if (LVM_BT_POINT_CLOUD == dev->capture_param->capture_data_type)
					{
						freqType = LVM_BT_POINT_CLOUD;
					}
					else if (LVM_BT_DEPTH_MAP == dev->capture_param->capture_data_type)
					{
						if (bm->data_mode == 3)
						{//3 深度图亮度图
							freqType = LVM_BT_DEPTH_INTENSITY;
						}
						else
						{//1深度图   2亮度图
							freqType = LVM_BT_DEPTH_MAP;
						}
					}
					((middleware_t *)dev->hw)->md.fps = lvm_get_dev_max_frame_rate(dev, freqType);

					data_frame * df = nullptr;
					if (((middleware_t *)dev->hw)->plugin_enable)
					{
						if (dev->capture_param->capture_data_type == LVM_BT_DEPTH_MAP
							&& ((middleware_t *)dev->hw)->net_dev->working_para.pcld.data_mode != 2)
						{
							list<plugin_obj> objs = get_dev_current_tasks(dev->dev_info->sn);
							if (objs.size() != 0)
							{
								lvm_depth_map_t *dm = (lvm_depth_map_t*)head_cp;

								//cv::Mat m_img = cv::Mat(dm->head.height, dm->head.width, CV_16UC1);
								//memcpy(m_img.data, dm->data, dm->head.height*dm->head.width * sizeof(ushort));

								df = new data_frame(((middleware_t *)dev->hw)->md.fid, DF_DEPTH, dm->head.width, dm->head.height,
									((middleware_t *)dev->hw)->md, dm->data, 0);
								df->axis_calib.dir[0] = dev->calib_param->dir[0];
								df->axis_calib.dir[1] = dev->calib_param->dir[1];
								df->axis_calib.dir[2] = dev->calib_param->dir[2];

								df->metadata.x_min = dev->depth_map_param->x_min;
								df->metadata.x_max = dev->depth_map_param->x_max;
								df->metadata.interpolation_len_max = dev->depth_map_param->interpolation_len_max;

								//m_img = cv::Mat(df->h, df->w, CV_16UC1);
								//memcpy(m_img.data, df->data, df->h*df->w * sizeof(ushort));

								processAndChangeDataFrameWithPlugin(dev->dev_info->sn, df);

								//m_img = cv::Mat(df->h, df->w, CV_16UC1);
								//memcpy(m_img.data, df->data, df->h*df->w * sizeof(ushort));

								//delete df;
							}
						}						
					}

					if (dev->capture_param->capture_data_type == LVM_BT_POINT_CLOUD)
					{
						//点云采集模式下将点云数据中的强度图解析到16位的数据结构中（非高速模式和同步采集模式均会做同样处理，数据源会变成整个缓存帧）
						lvm_point_t *point = (lvm_point_t*)buf_frame_line_at(buf, frame, bm->r_off);
						for (int i = 0; i < frame->width * lines; i++)
						{
							point[i].reserved = (point[i].reserved & 0xFFC00000) >> 16;
						}

						if (!((middleware_t *)dev->hw)->section_enable)
							frame_cb_img_callback(dev, head_cp, point, lines);	//不设置段情况。按常规高速模式的形式发送
						else
						{
							lvm_point_cloud_section_split(dev, (lvm_point_cloud_t*)head_cp, point, lines);
						}
					}
					else if (dev->capture_param->capture_data_type == LVM_BT_IMAGE)
					{
						frame_cb_img_callback(dev, head_cp, buf_frame_line_at(buf, frame, bm->r_off), lines);
					}
					else
					{
						if (df)
						{
							lvm_depth_map_t tmp;
							lvm_depth_map_t *dm = (lvm_depth_map_t*)head_cp;
							memcpy(&tmp.head, &dm->head, sizeof(lvm_frame_head_t));

							tmp.head.width = df->w;
							tmp.head.height = df->h;
							tmp.x_offset = df->metadata.x_offset;
							tmp.y_offset = df->metadata.y_offset;

							lvm_depth_map_param_t tmp_param;
							memcpy(&tmp_param, dm->param, sizeof(lvm_depth_map_param_t));

							tmp_param.x_min = df->metadata.x_min;
							tmp_param.x_max = df->metadata.x_max;

							tmp.param = &tmp_param;
							tmp.data = df->data;

							//lvm_frame_head_t head;

							//int x_offset;                                   //x方向偏移
							//unsigned long long y_offset;                    //y方向偏移
							//lvm_depth_map_param_t *param;
							//void *data = nullptr;
							if ((((middleware_t *)dev->hw)->net_dev->working_para.peripheral.profile.profile_enable) && (df->data != nullptr)) //单行轮廓仪数据处理
							{    
								//保存参数
								lvm_depth_map_param_t origin_param = *(dm->param);
								double origin_x_offset = dm->x_offset;

								lvm_profile_data_proc(dev, &tmp, df->data, df->w, df->h);
								frame_cb_img_callback(dev, &tmp, nullptr, lines);
								//还原参数        ==========>这里可能有问题
								*(tmp.param) = origin_param;
								dm->x_offset = origin_x_offset;
							}
							else
							{
								frame_cb_img_callback(dev, &tmp, nullptr, lines);
							}
							delete df;
						}
						else
						{
                            if(((middleware_t *)dev->hw)->net_dev->working_para.pcld.data_mode != 2)
                            {
								//cout << "\tcurrent intensity:" << ((lvm_depth_map_t*)head_cp)->intensity_img->head.height << "\tcurrent depth:" << ((lvm_depth_map_t*)head_cp)->head.height << endl;
								if (!((middleware_t *)dev->hw)->section_enable)
								{							
									if (((middleware_t *)dev->hw)->net_dev->working_para.peripheral.profile.profile_enable) //单行轮廓仪数据处理
									{
										//保存参数
										lvm_depth_map_param_t origin_param = *(((lvm_depth_map_t*)head_cp)->param);
										double origin_x_offset = ((lvm_depth_map_t*)head_cp)->x_offset;

										lvm_depth_map_t *depth_map = (lvm_depth_map_t*)head_cp;
										lvm_profile_data_proc(dev, head_cp, depth_map->data, depth_map->head.width, depth_map->head.height);
										frame_cb_img_callback(dev, head_cp, buf_frame_line_at(buf, frame, bm->r_off), lines);

										//还原参数        ==========>这里可能有问题
										*(((lvm_depth_map_t*)head_cp)->param) = origin_param;
										((lvm_depth_map_t*)head_cp)->x_offset = origin_x_offset;

									}
									else
									{
										frame_cb_img_callback(dev, head_cp, buf_frame_line_at(buf, frame, bm->r_off), lines);
									}
								}
    							else
    							{       //高速深度图
    								lvm_depth_map_section_split(dev, (lvm_depth_map_t*)head_cp, buf_frame_line_at(buf, frame, bm->r_off), lines);
    							}
                            }
                            else
                            {
								frame_cb_img_callback(dev, head_cp, buf_frame_line_at(buf, frame, bm->r_off), lines);
                            }
                        }
                    }
                }
				else
				{
					lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"hs mode data drop %d lines because of no CB\n", lines);
				}
				//wr_locker.lock();

				bm->r_off += lines;
				bm->r_lnum += lines;

				if (frame_jump)
				{
					bm->r_id++;
					if (bm->r_id == buf->frame_num)
						bm->r_id = 0;
					bm->r_fnum++;
					bm->r_off = 0;
				}
			}
			while ((bm->r_lnum == bm->w_lnum) && (bm->r_fnum == bm->w_fnum) && bm->read_thread_run)
			{
				bm->cond_write.wait(wr_locker); //wait conditional variable
			}
		}
		free(head_cp);
	}
	else
		/*
		>>> normal mode:
		if r_fnum<w_fnum, read one buffer frame, r_id
		r_id++, r_fnum++, r_off reset;
		*/
	{
		while (bm->read_thread_run)
		{
			while ((bm->r_fnum != bm->w_fnum) && bm->read_thread_run)
			{
				lvm_frame_head_t *frame = (lvm_frame_head_t*)buf_frame_at(buf, bm->r_id);
				bm->r_off = 0;
				int lines = frame->height;

                //frame->flags = 0;
                ////junhui add non hs_mode eof
                //frame->flags |= FRAME_EOF;
				/* in non high speed mode, the small frames not at the beginning should not have SOF set
				prevent SOF storm confusing users */

				//log_put(dev, DBG_INFO_EN, LVM_INFO_EVENT, "rt:unlock\n");
				wr_locker.unlock();
				if (dev->frame_cb)
				{
					if (1 == ((middleware_t *)dev->hw)->stop_flag)
					{
						return -1;
					}
					if (dev->capture_param->capture_data_type == LVM_BT_DEPTH_MAP
						&& ((middleware_t *)dev->hw)->net_dev->working_para.pcld.data_mode != 2)
					{
						lvm_depth_map_t *dm = (lvm_depth_map_t*)frame;
		/*				alg_proc_data(dev, bm->uniformity_enable, bm->intensity_enable, frame,
							buf_frame_line_at(buf, frame, bm->r_off),0,0, lines);*/
						//
						if (1 == ((middleware_t *)dev->hw)->xz_compen_enable)
						{
							lvm_point_t *point = (lvm_point_t*)malloc(dm->head.width*dm->head.height * sizeof(lvm_point_t));
							dm_to_point(dm, point);
							vector<double> compen_param; compen_param.clear();
							vector<double> laser_param; laser_param.clear();
							vector<double> transf_param; transf_param.clear();
							for (int i = 0; i < 5; i++)
							{
								compen_param.push_back(((middleware_t *)dev->hw)->net_dev->device_para.part2.Compen[i] +
									((middleware_t *)dev->hw)->net_dev->working_para.peripheral.laser.Compen[i]);
							}
							for (int i = 0; i < 3; i++)
							{
								laser_param.push_back(((middleware_t *)dev->hw)->net_dev->device_para.part2.Laser[i]);
								transf_param.push_back(((middleware_t *)dev->hw)->net_dev->device_para.part2.T[i]);
							}

							compensatePoints(point, dm, dm->head.width, dm->head.height, laser_param, transf_param, compen_param);
							free(point);
							point = NULL;
						}

						if (dm->param->x_scale != 0 && dm->param->y_scale != 0)
						{
							if (!median_filter(dm->data, dm->head.width, dm->head.height, 
								dev->config_param->median_x / dm->param->x_scale, dev->config_param->median_y / dm->param->y_scale,
								((middleware_t *)dev->hw)->stop_flag))
							{
								lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"median filter invalid or failed\n");
							}
							if (!average_filter(dm->data, dm->head.width, dm->head.height, 
								dev->config_param->average_x / dm->param->x_scale, dev->config_param->average_y / dm->param->y_scale,
								((middleware_t *)dev->hw)->stop_flag))
							{
								lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"average filter invalid or failed\n");
							}
							if (!null_filler_filter(dm->data, dm->head.width, dm->head.height,
								dev->config_param->null_filler_x / dm->param->x_scale, dev->config_param->null_filler_y / dm->param->y_scale,
								((middleware_t *)dev->hw)->stop_flag))
							{
								lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"null filter invalid or failed\n");
							}
						}

						if (1 == ((middleware_t *)dev->hw)->corr_enable)
						{
							lvm_point_t *point = (lvm_point_t*)malloc(dm->head.width*dm->head.height * sizeof(lvm_point_t));
							dm_to_point(dm, point);
							vector<float> rw; rw.clear();
							vector<float> tw; tw.clear();
							float x_min = 0.0f, x_max = 0.0f;
							for (int i = 0; i < 9; i++)
							{
								rw.push_back(((middleware_t *)dev->hw)->world_rotation_backup[i]);
							}
							for (int i = 0; i < 3; i++)
							{
								tw.push_back(((middleware_t *)dev->hw)->world_translation_backup[i]);
							}
							angleCorrection(point, dm->head.width, dm->head.height, rw, tw, x_min, x_max);
							if (x_min >= dm->param->x_min && x_max <= dm->param->x_max)
							{
								point_to_dm(point, dm);
							}
							else
							{
								lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE, "x_min&x_max range error!!!!\n");
							}
							free(point);
							point = NULL;
						}
						((middleware_t *)dev->hw)->md.x_offset = dev->depth_map_param->x_scale*dm->x_offset;
						((middleware_t *)dev->hw)->md.y_offset = dev->depth_map_param->y_scale*dm->y_offset;
					}
					else if (dev->capture_param->capture_data_type == LVM_BT_POINT_CLOUD)
					{
						lvm_point_cloud_t *pcld = (lvm_point_cloud_t*)frame;

	/*					alg_proc_data(dev, bm->uniformity_enable, bm->intensity_enable, frame,
							buf_frame_line_at(buf, frame, bm->r_off), 
							pcld_dm_buf_frame_line_at(buf, frame, pcld->dm->head.width * sizeof(ushort), bm->r_off),
							pcld_intensity_img_buf_frame_line_at(buf, frame, pcld->insensity_img->head.width * sizeof(ushort), bm->r_off),
							lines);*/
						if (1 == ((middleware_t *)dev->hw)->xz_compen_enable)
						{
							vector<double> compen_param; compen_param.clear();
							vector<double> laser_param; laser_param.clear();
							vector<double> transf_param; transf_param.clear();
							for (int i = 0; i < 5; i++)
							{
								compen_param.push_back(((middleware_t *)dev->hw)->net_dev->device_para.part2.Compen[i] +
									((middleware_t *)dev->hw)->net_dev->working_para.peripheral.laser.Compen[i]);
							}
							for (int i = 0; i < 3; i++)
							{
								laser_param.push_back(((middleware_t *)dev->hw)->net_dev->device_para.part2.Laser[i]);
								transf_param.push_back(((middleware_t *)dev->hw)->net_dev->device_para.part2.T[i]);
							}
							lvm_depth_map_t *dm_data = NULL;
							compensatePoints(pcld->p, dm_data, pcld->head.width, pcld->head.height, laser_param, transf_param, compen_param);
						}

						if (bm->uniformity_enable)
						{
							cv::Mat dst, intensity_dst;
							dst.release();
							intensity_dst.release();
							
							dst = cv::Mat(lines, pcld->dm->head.width, CV_16UC1, pcld_dm_buf_frame_line_at(buf, frame, pcld->dm->head.width * sizeof(ushort), bm->r_off));
							memset(dst.data, 0, dst.cols*dst.rows * sizeof(ushort));
							pcld->dm->head.height = lines;
							pcld_to_depth_map(pcld->p, pcld->head.width, pcld->head.height, dst, *pcld->dm->param);

							if (bm->intensity_enable)
							{
								//get intensity img
								intensity_dst = cv::Mat(pcld->head.height, pcld->dm->head.width, CV_16UC1, pcld_intensity_img_buf_frame_line_at(buf, frame, pcld->insensity_img->head.width * sizeof(ushort), bm->r_off));
								memset(intensity_dst.data, 0, intensity_dst.cols*intensity_dst.rows * sizeof(ushort));
								pcld->insensity_img->head.height = lines;
								pcld_to_intensity_map(pcld->p, pcld->head.width, pcld->head.height, intensity_dst, *pcld->dm->param);
							}

							if (pcld->dm->param->x_scale != 0 && pcld->dm->param->y_scale != 0)
							{
		
								if (!median_filter(pcld->dm->data, pcld->dm->head.width, pcld->dm->head.height,
									dev->config_param->median_x / pcld->dm->param->x_scale, dev->config_param->median_y / pcld->dm->param->y_scale,
									((middleware_t *)dev->hw)->stop_flag))
								{
									lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"median filter invalid or failed\n");
								}
								if (!average_filter(pcld->dm->data, pcld->dm->head.width, pcld->dm->head.height, 
									dev->config_param->average_x / pcld->dm->param->x_scale, dev->config_param->average_y / pcld->dm->param->y_scale,
									((middleware_t *)dev->hw)->stop_flag))
								{
									lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"average filter invalid or failed\n");
								}
								if (!null_filler_filter(pcld->dm->data, pcld->dm->head.width, pcld->dm->head.height,
									dev->config_param->null_filler_x / pcld->dm->param->x_scale, dev->config_param->null_filler_y / pcld->dm->param->y_scale,
									((middleware_t *)dev->hw)->stop_flag))
								{
									lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"null filter invalid or failed\n");
								}
							}

							if (1 == ((middleware_t *)dev->hw)->corr_enable)
							{
								lvm_point_t *point = (lvm_point_t*)malloc(pcld->dm->head.width*pcld->dm->head.height * sizeof(lvm_point_t));
								dm_to_point(pcld->dm, point);
								vector<float> rw; rw.clear();
								vector<float> tw; tw.clear();
								float x_min = 0.0f, x_max = 0.0f;
								for (int i = 0; i < 9; i++)
								{
									rw.push_back(((middleware_t *)dev->hw)->world_rotation_backup[i]);
								}
								for (int i = 0; i < 3; i++)
								{
									tw.push_back(((middleware_t *)dev->hw)->world_translation_backup[i]);
								}
								angleCorrection(point, pcld->dm->head.width, pcld->dm->head.height, rw, tw, x_min, x_max);
								if (x_min >= pcld->dm->param->x_min && x_max <= pcld->dm->param->x_max)
								{
									point_to_dm(point, pcld->dm);
								}
								else
								{
									lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE, "x_min&x_max range error!!!!\n");
								}
								free(point);
								point = NULL;

								
							}
							((middleware_t *)dev->hw)->md.x_offset = dev->depth_map_param->x_scale*pcld->dm->x_offset;
							((middleware_t *)dev->hw)->md.y_offset = dev->depth_map_param->y_scale*pcld->dm->y_offset;

							if (((middleware_t *)dev->hw)->plugin_enable)
							{
								list<plugin_obj> objs = get_dev_current_tasks(dev->dev_info->sn);
								if (objs.size() != 0)
								{
									data_frame * df = new data_frame(((middleware_t *)dev->hw)->md.fid, DF_DEPTH, pcld->dm->head.width, pcld->dm->head.height,
										((middleware_t *)dev->hw)->md, pcld->dm->data, 0);

									df->axis_calib.dir[0] = dev->calib_param->dir[0];
									df->axis_calib.dir[1] = dev->calib_param->dir[1];
									df->axis_calib.dir[2] = dev->calib_param->dir[2];

									df->metadata.x_min = dev->depth_map_param->x_min;
									df->metadata.x_max = dev->depth_map_param->x_max;
									df->metadata.interpolation_len_max = dev->depth_map_param->interpolation_len_max;

									processAndChangeDataFrameWithPlugin(dev->dev_info->sn, df);

									pcld->dm->head.width = df->w;
									pcld->dm->head.height = df->h;
									pcld->dm->x_offset = df->metadata.x_offset;
									pcld->dm->y_offset = df->metadata.y_offset;
									pcld->dm->param->x_min = df->metadata.x_min;
									pcld->dm->param->x_max = df->metadata.x_max;

									memcpy(pcld->dm->data, df->data, df->w*df->h * 2);

									delete df;
								}
							}
						}
						else
						{
							if (1 == ((middleware_t *)dev->hw)->corr_enable)
							{
								vector<float> rw; rw.clear();
								vector<float> tw; tw.clear();
								float x_min = 0.0f, x_max = 0.0f;
								for (int i = 0; i < 9; i++)
								{
									rw.push_back(((middleware_t *)dev->hw)->world_rotation_backup[i]);
								}
								for (int i = 0; i < 3; i++)
								{
									tw.push_back(((middleware_t *)dev->hw)->world_translation_backup[i]);
								}
								angleCorrection(pcld->p, pcld->head.width, pcld->head.height, rw, tw, x_min, x_max);
							}
						}
					}

					if (1 == ((middleware_t *)dev->hw)->stop_flag)
					{
						return -1;
					}

					//填充metadata信息
					time_t cur_time = time(0);
					struct tm tm;
					tm = *localtime(&cur_time);
					strftime(((middleware_t *)dev->hw)->md.time, sizeof(((middleware_t *)dev->hw)->md.time), "%Y:%m:%d %H:%M:%S", &tm);
					//strcpy(p->md.time, asctime(localtime(&cur_time)));
					((middleware_t *)dev->hw)->md.points_of_line = frame->width;
					((middleware_t *)dev->hw)->md.grab_lines = frame->height;
					((middleware_t *)dev->hw)->md.x_scale = dev->depth_map_param->x_scale;
					((middleware_t *)dev->hw)->md.y_scale = dev->depth_map_param->y_scale;
					((middleware_t *)dev->hw)->md.z_scale = dev->depth_map_param->z_scale;

					((middleware_t *)dev->hw)->md.z_offset = dev->depth_map_param->z_min;
					((middleware_t *)dev->hw)->md.fid = frame->fid;
					((middleware_t *)dev->hw)->md.sid = frame->sid;
					((middleware_t *)dev->hw)->md.time_stamp = frame->time_stamp;
					((middleware_t *)dev->hw)->md.lost_lines = frame->lost_lines;
					((middleware_t *)dev->hw)->md.trigger_min_interval = frame->trigger_min_interval;
					((middleware_t *)dev->hw)->md.trigger_max_interval = frame->trigger_max_interval;
					((middleware_t *)dev->hw)->md.flags = frame->flags;
					((middleware_t *)dev->hw)->md.temperature_j28 = dev->status->temperature_j28;
					((middleware_t *)dev->hw)->md.temperature_j29 = dev->status->temperature_j29;
					((middleware_t *)dev->hw)->md.temperature_j30 = dev->status->temperature_j30;
					lvm_buf_type_t freqType = LVM_BT_IMAGE;
					if (LVM_BT_POINT_CLOUD == dev->capture_param->capture_data_type)
					{
						freqType = LVM_BT_POINT_CLOUD;
					}
					else if (LVM_BT_DEPTH_MAP == dev->capture_param->capture_data_type)
					{
						if (bm->data_mode == 3)
						{//3 深度图亮度图
							freqType = LVM_BT_DEPTH_INTENSITY;
						}
						else
						{//1深度图   2亮度图
							freqType = LVM_BT_DEPTH_MAP;
						}
					}
					((middleware_t *)dev->hw)->md.fps = lvm_get_dev_max_frame_rate(dev, freqType);

					if (((middleware_t *)dev->hw)->plugin_enable)
					{
						if (dev->capture_param->capture_data_type == LVM_BT_DEPTH_MAP
							&& ((middleware_t *)dev->hw)->net_dev->working_para.pcld.data_mode != 2)
						{
							list<plugin_obj> objs = get_dev_current_tasks(dev->dev_info->sn);
							if (objs.size() != 0)
							{
								lvm_depth_map_t *dm = (lvm_depth_map_t*)frame;
								data_frame * df = new data_frame(((middleware_t *)dev->hw)->md.fid, DF_DEPTH, frame->width, frame->height,
									((middleware_t *)dev->hw)->md, dm->data, 0);

								df->axis_calib.dir[0] = dev->calib_param->dir[0];
								df->axis_calib.dir[1] = dev->calib_param->dir[1];
								df->axis_calib.dir[2] = dev->calib_param->dir[2];

								df->metadata.x_min = dev->depth_map_param->x_min;
								df->metadata.x_max = dev->depth_map_param->x_max;
								df->metadata.interpolation_len_max = dev->depth_map_param->interpolation_len_max;

								processAndChangeDataFrameWithPlugin(dev->dev_info->sn, df);

								dm->head.width = df->w;
								dm->head.height = df->h;
								dm->x_offset = df->metadata.x_offset;
								dm->y_offset = df->metadata.y_offset;
								dm->param->x_min = df->metadata.x_min;
								dm->param->x_max = df->metadata.x_max;

								memcpy(dm->data, df->data, df->w*df->h * 2);

								delete df;
							}							
						}
					}

					if (dev->capture_param->capture_data_type == LVM_BT_POINT_CLOUD)
					{
						lvm_point_cloud_t *cb_frame = (lvm_point_cloud_t*)frame;
						for (int i = 0; i < frame->width * frame->height; i++)
						{
							cb_frame->p[i].reserved = (cb_frame->p[i].reserved & 0xFFC00000) >> 16;
						}
						frame_cb_img_callback(dev, cb_frame, buf_frame_line_at(buf, frame, bm->r_off), lines);
					}
					else
					{
						if (((middleware_t *)dev->hw)->net_dev->working_para.peripheral.profile.profile_enable) //单行轮廓仪数据处理
						{
							lvm_depth_map_t *depth_map = (lvm_depth_map_t*)frame;

							//保存参数
							lvm_depth_map_param_t origin_param = *(depth_map->param);
							double origin_x_offset = depth_map->x_offset;

							lvm_profile_data_proc(dev,  frame, depth_map->data,depth_map->head.width, depth_map->head.height);
							frame_cb_img_callback(dev, frame, buf_frame_line_at(buf, frame, bm->r_off), lines);

							//还原参数        ==========>这里可能有问题
							*(depth_map->param) = origin_param;
							depth_map->x_offset = origin_x_offset;
						}
						else
						{ 
							frame_cb_img_callback(dev, frame, buf_frame_line_at(buf, frame, bm->r_off), lines);
						}
					}
				}
				else
				{
					// tell user one frame in read thread is ready
					rt_ready_locker.lock();
					bm->sync_ready_frame = frame;
					bm->sync_done = 0;
					bm->cond_rt_frame_ready.notify_one();

					// wait user to release it
					while (((bm->sync_ready_frame != 0) || (bm->sync_done == 1)) && bm->read_thread_run)
						bm->cond_user_frame_done.wait(rt_ready_locker); //wait user read one frame done
					rt_ready_locker.unlock();
				}
				//printf("rt:lock\n");


				wr_locker.lock();

				bm->r_lnum += lines;
				bm->r_id++;
				if (bm->r_id == buf->frame_num)
					bm->r_id = 0;
				bm->r_fnum++;
			}
			while ((bm->r_fnum == bm->w_fnum) && bm->read_thread_run)
			{
				//printf("rt:wait\n");
				bm->cond_write.wait(wr_locker); //wait conditional variable
			}
		}
	}

	lvm_put_event(dev, DBG_INFO_EN, LVM_INFO_EVENT, DBG_INFO_CODE,"rd_thread quit\n");
	return 0;
}