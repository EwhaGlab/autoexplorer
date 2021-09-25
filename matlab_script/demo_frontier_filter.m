
rosinit

map_sub = rossubscriber('/tesse0/projected_map') ;
costmap_sub = rossubscriber('/tesse0/move_base_node/global_costmap/costmap') ;
centroid_sub = rossubscriber('/tesse0/centroids') ;

map = receive(map_sub,10) ;
costmap = receive(costmap_sub,10) ;
centroids = receive(centroid_sub,10) ;