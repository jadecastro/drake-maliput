#! /bin/sh

./automotive_demo.py -road_file ./maliput/utility/village.yaml \
		     -num_ado_car=2 -num_ego_car=1 -num_user_car=0 -use_idm \
		     -road_path start:l:Express_WN_a,l:Express_WN_b,l:Express_N1,l:Express_NRamp,l:Y-Ave_isA,l:Y-Ave_blk1,l:B1-Y1,l:B-St_blk1,l:B-St_isX,l:Express_WRamp_a,l:Express_WRamp_b,l:Express_WN_a \
		     -road_path_traffic start:l:Express_WN_a,l:Express_WN_b,l:Express_N1,l:Express_NRamp,l:Y-Ave_isA,l:Y-Ave_blk1,l:Y-Ave_isB,l:Y-Ave_blk2,l:Y-Ave_isC,l:Express_SRamp_a,l:Express_SRamp_b,l:Express_S2,l:Express_SW_a,l:Express_SW_b,l:Express_W_a,l:Express_W_b,l:Express_WN_a
