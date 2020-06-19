
* Save map
 
```
$ rosrun map_server map_saver -f mymap
```

```
$ rosservice call /hdl_graph_slam/save_map "resolution: 0.05 destination: '~/map.pcd'"
```