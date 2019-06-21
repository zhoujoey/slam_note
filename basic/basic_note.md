# Basic Note

## 文章1
ORB_SLAM中使用到g2o进行优化的函数有四个: 
GlobalBundleAdjustemnt 
PoseOptimization 
LocalBundleAdjustment 
OptimizeEssentialGraph 
OptimizeSim3.

下面对这几个不同的函数进行逐一分析:

1. GlobalBundleAdjustment

这个函数的作用是对全局的地图点和keyframe位姿进行优化. 即大图优化, 非常费时. 
(1) 提取出所有的keyframes和所有的地图点. 
(2) 把keyframe设置为图中的节点 
(3) 把每一个地图点设置为图中的节点, 然后对于每一个地图点, 找出所有能看到这个地图点的keyframe. 
(4) 对每一个keyframe, 建立边: 
边的两端分别是地图点的SE3位姿与当前keyframe的SE位姿. 
边的观测值为该地图点在当前keyframe中的二维位置. 
信息矩阵(权重)是观测值的偏离程度, 即3D地图点反投影回地图的误差.

(5) 构图完成, 进行优化. 
(6) 把优化后的地图点和keyframe位姿全部放回原本的地图中.

2. PoseOptimization

对于每一个keyframe, 进行位姿的局部优化. 
(1) 直接把当前keyframe设为地图中的节点 
(2) 找出所有在当前keyframe中可见的的三维地图点, 对每一个地图点, 建立边: 
边的两端分别是keyframe的位姿与当前地图点为位姿. 
边的观测值为该地图点在当前keyframe中的二维位置. 
信息矩阵(权重)是观测值的偏离程度, 即3D地图点反投影回地图的误差. 
(3) 构图完成,进行优化. 这里的优化要做四次, 没搞明白为什么. 
(4) 把优化后的keyframe位姿放回去.

3. LocalBundleAdjustment

(1) 找到Local Keyframe, 即那些共享CovisbilityMap的Keyframes. 存入lLocalKeyFrames. 
(2) 找到所有Local Keyframes都能看到的地图点, 其实就是CovisbilityMap的地图点. 存入lLocalMapPoints. 
(3) 再找出能看到上面的地图点, 却不在Local Keyframe范围内的keyframe(为什么??). 存入lFixedCameras. 
(4) 把上面的Local Keyframe, Map Point, FixedCamera都设置为图节点. 
(5)对于lLocalMapPoints中的每一个地图点及能看到它的所有keyframes, 建立边: 
边的两端分别是keyframe的位姿与当前地图点为位姿. 
边的观测值为该地图点在当前keyframe中的二维位置. 
信息矩阵(权重)是观测值的偏离程度, 即3D地图点反投影回地图的误差. 
(6) 去除掉一些不符合标准的边. 
(7) 把优化后地图点和keyframe位姿放回去.

4. OptimizeEssentialGraph

这个函数应该是这几个之中最重要的一个. 
(1) 首先获取所有keyframes和地图点. 
(2) 把keyframe都设为图节点, 剔除不好的keyframe. 这里的frame位姿为Sim3. 
(3) 添加边 
<1> Loop edge: LoopConnections 是一个Map, Map中第一个元素是有loop的keyframe, 第二个元素是与第一个元素形成loop的keyframe集合. 给它们全部添加边进行连接. 
边的观测值是后一帧的SIM3位姿乘以前一帧的SIM3位姿的逆. (为什么??) 
<2> Normal edge: 遍历所有的keyframe, 
- 找到当前keyframe的parent keyframe, 建立边连接. 边的观测值为parent keyframe的位姿乘以keyframe位姿的逆(为什么??). 信息矩阵为单位矩阵. 
- 找到与当前keyframe形成Loop的所有keyframe, 如果找到成Loop的keyframe在当前keyframe之前, 则在两个keyframe之间添加一个边连接. 观测值为Loop keyframe的位姿乘以keyframe位姿的逆(为什么??). 信息矩阵为单位矩阵. 
- 找到当前keyframe的covisibility graph 中的每一个keyframe, 建立边连接. 观测值为covisibility graph keyframe位姿乘以keyframe位姿的逆(为什么??). 信息矩阵为单位矩阵.

(4) 构图完成, 进行优化. 
(5) 更新EssentialGraph中的所有位姿.

5. OptimizeSim3

(1) 把输入的KF1到KF2的位姿变换SIM3加入图中作为节点0. 
(2) 找到KF1中对应的所有map点, 放在vpMapPoints1中. vpMatches1为输入的匹配地图点, 是在KF2中匹配上map点的对应集合. 
(3) Point1是KF1的Rotation matrix*map point1的世界坐标+KF1的Translation matrix. Point2是KF2的Rotation matrix*map point2的世界坐标+KF2的Translation matrix. 
把Point1 Point2作为节点加入图中 
(4) 在节点0与Point1, Point2之间都建立边连接. 测量值分别是地图点反投影在图像上的二维坐标, 信息矩阵为反投影的误差. 
(5) 图构建完成, 进行优化! 
(6) 更新两帧间转换的Sim3.

总结一下, ORB_SLAM中的优化分为多个层次, 从低到高为:

PoseOptimization: 优化单帧位姿. 每个帧可见多个地图点, 可以建立多个边连接, 构成图进行优化. 优化单一帧的SE3位姿.
OptimizeSim3: 优化两帧之间的位姿变换, 因为两帧之间可以看到多个相同的地图点, 可以构成一个超定方程组, 可以最小化误差优化. 优化帧间变化的SIM3位姿与地图的VertexSBAPointXYZ位姿
LocalBundleAdjustment: 在一个CovisbilityMap内进行优化. 在一定范围的keyframe中,可以看到同一块地图, 即是CovisbilityMap. 连接CovisbilityMap内的每一个map点与可见它的所有keyframe, 放在一起进行优化. 这个优化是双向的, 既优化地图点的VertexSBAPointXYZ位姿, 又优化Camera的SE3位姿.
OptimizeEssentialGraph: 加入Loop Closure的考虑, Covisbility图中的keyframe相互连起来, Keyframe之间有前后相连, 于是不同的Covisbility图也可以联系起来. 这是一个大范围的优化, 主要是加入了Loop Closure约束. 优化的是Camera的SIM3位姿.
GlobalBundleAdjustment: 最大范围的优化, 优化所有Camera的SE3位姿与地图点的XYZ位姿.

