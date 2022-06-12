# SOFA-based-retinal-surgery-simulation-system

## Description:

  [Paper link](https://www.overleaf.com/project/629b234da599cd10446b7e94) (Temporary)

  For now only Windows OS is supported.
  
## Dependencies:
  1. Download and install SOFA framework(windows prebuilt distribution [v19.06](https://github.com/sofa-framework/sofa/releases?page=2) ).
  2. Follow the instrution of installation of SOFA dependencies( [Visual C++ Redistributable Packages for Visual Studio 2015](https://www.microsoft.com/en-us/download/details.aspx?id=53587) and 
[Python 2.7 (amd64)](https://www.python.org/ftp/python/2.7.16/python-2.7.16.amd64.msi))
  3. Make sure it is accessible in the system PATH. For windows, it is located at environment variables. See tutorial [here](https://datascience.com.co/how-to-install-python-2-7-and-3-6-in-windows-10-add-python-path-281e7eae62a)
  4. Download all the files in this repo and unzip. Make sure that they are in the same folder
  5. Download the models via the google drive [link](https://drive.google.com/file/d/1rv4pEZWJ5c9ynYa9-Uq_jwFuEMhzF2js/view?usp=sharing) and unzip into the same folder with the repo.


## Task Description:
In this simulated scenario, trainees were assigned a retinal vascular injection task. 

A total of two blood vessel branches need to be injected with thrombolytics by trainees, which are marked with green spheres in the scene.
By operating the keyboard and handle ([SpaceMouse](https://3dconnexion.com/uk/spacemouse/)) respectively, the trainee can control the four-degree-of-freedom movement of the injection needle in the simulation environment (three rotations, one transmission). 
Move the needle to the branch of the blood vessel where the thrombolytic agent needs to be injected, and wait for one second to complete the injection. 
At this time, the ball turns red, which means that the thrombolysis is successful and the blood vessel is healed and turns red. 

When two vascular injection tasks are completed separately, the entire task is considered complete. 
Throughout the process, the simulation environment detects whether the needle collides with other parts of the retina, causing trauma. When all tasks are completed, the evaluation system will judge the quality of the surgical operation, and give feedback and score based on the operation time and the number of trauma. 

We hope that such a simulated experimental task can not only improve the proficiency and confidence of ophthalmologists in performing retinal surgery with bare hands, 
but also change the mapping relationship of control, so that doctors can be more familiar with the operation of the MicroFeng ophthalmic robot, so that retinal surgery can benefit from the robot the advantages of high-precision motion.

在这个模拟场景中，练习生被assigned视网膜血管注射任务。

总共两处血管分支需要被练习生注射溶栓剂，在场景中用绿色球面标出。分别通过操作键盘和手柄([SpaceMouse](https://3dconnexion.com/uk/spacemouse/)), 练习生可以控制仿真环境中注射针的四自由度运动（三个转动，一个传动）。
分别将针头移动到需要被注射溶栓剂的血管分支中，静待一秒钟完成注射，此时圆球变红色，代表溶栓成功，血管痊愈变成红色。分别完成两个血管注射任务，则整个任务被视为完成。

在整个过程中，仿真环境会检测针和视网膜其他位置是否进行碰撞，产生创伤。全部任务完成时，评价系统会对手术操作质量做出判断，并且基于操作时间和创伤数量给出反馈、进行打分。

我们希望如此的模拟实验任务不仅可以提高眼科医生徒手进行视网膜手术的熟练度和自信心，同时改变控制的映射关系，可以使医生对微锋眼科机器人的操作更加熟悉，使视网膜手术可以受益于机器人的高精度运动的优点。


## Keyboard Version:
  1. Load the script /scripts/scene/TotalScene.py into SOFA interface.
  2. Click the 'Run' button and the whole simulation will be on the fly.
  3. You can always control the needle with number keys 1 3, 4 6, 7 9 for three different rotation degrees. 
And number keys 8 2 for translation degree, say "insert" or "draw" in this scenario. 
  4. For control the speed or the stepsize of motion, you can always click key "k" or "m" to speed up or slow down your motions.
In case you can't remember it, "k" for "kuai" and "m" for "man" separately in Chinese.
  5. To adjust the magnification of microscope, "PgUp" and "PgDn" can be pressed.
  6. See if you are a noob via checking whether the final points are beyond the passing line: 60 points.

  For an introduction video for Keyboard Control Demo please click [here](https://www.youtube.com/watch?v=-Gi7CAQmXC8&list=UUBhSckzAoAqAJ7lWwTK7VVg&index=2).


## SpaceMouse Version:
  1. Load the script /scripts/scene/TotalScene_space.py into SOFA interface.
  2. Click the 'Run' button and the whole simulation will be on the fly.
  3. You can always control the needle with the spacemouse for 4 degrees. Total degrees of SpaceMouse are 6 and the mapping controlling relations are listed following. 

  ![spacemosue](./imgs/dofs.jfif)
    
  Tilt, Spin and Roll are corresponding to three rotations separatly and Pan up/down are mapped into "insert"/"draw".
    
4. For control the speed or the stepsize of motion, you can always click key "k" or "m" to speed up or slow down your motions.
    In case you can't remember it, "k" for "kuai" and "m" for "man" separately in Chinese.
5. To adjust the magnification of microscope, "PgUp" and "PgDn" can be pressed.
6. Good news: since SpaceMouse is a bit more difficult than controlling with keyboard, 
you are only a noob if you could not pass 50 points(60 for keyboard).

  For an introduction video for Keyboard Control Demo please click [here](https://www.youtube.com/watch?v=gn6k4969WGo&list=UUBhSckzAoAqAJ7lWwTK7VVg&index=1).
