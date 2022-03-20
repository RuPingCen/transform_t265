
# transform_t265

从T265相机发布里程计数据的Topic上接收数据，转发为path类型，并存储为txt格式数据。详细的中文教程可以参考博客[Intel Realsense T265使用教程](https://blog.csdn.net/crp997576280/article/details/109544456?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522164777491716782246483640%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=164777491716782246483640&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-1-109544456.142^v2^pc_search_result_cache,143^v4^register&utm_term=T265&spm=1018.2226.3001.4187) 

## 1.1 下载与配置

 1. cd catkin_ws/src
 
 2. git clone  https://github.com/RuPingCen/transform_t265.git

 3. catkin_make
 
## 1.2 transform_t265 节点
 
     roslaunch transform_t265 transform_t265.launch

## 1.3 参数说明

sub_topic 接收里程计数据的话题 (nav_msgs::Odometry)

pub_topic 将里程计发布为path类型 (nav_msgs::Path)

is_save_path 是否保存轨迹

save_path 轨迹保存的路径

save_style 轨迹输出的类型 openvins tum euroc 数据类型


## 1.4 保存类型

openvins的类型格式符号为：timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33

```
    1647508752.67794 0.010931 -0.018046 -0.012423 0.858526 0.001739 -0.012059 0.512625 0.0022190017 0.0000197923 -0.0000106589 0.0014448213 0.0004173168 0.0019947021 0.0558263810 0.0000000313 -0.0000654889 0.0558263845 0.0001009220 0.0263604116
```

TUM数据的类型为：time x y z qx qy qz qw（一共8列，中间用空格间隔，时间单位为秒，保留小数点后9位）

```
    1645862863.690412521 -0.02391 0.03080 0.00446 -0.00861 0.31453 0.00753 0.94918
```


EUROC数据的类型为：time x y z qw qx qy qz （与TUM不同的是 EUROC把 qw 放在前）

```
    1645862863.690412521 -0.02391 0.03080 0.00446 0.94918 -0.00861 0.31453 0.00753
```




