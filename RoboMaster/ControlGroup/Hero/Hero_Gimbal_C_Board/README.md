# RM2023-Board-C-StdPeriph

#### 介绍
​	23赛季步兵电控C板代码仓库

- ​	新增USB CDC外设，从此小电脑和32的连接无需串口TTL模块，仅需一根Micro USB线。

#### 拉取代码方法
1. 随便打开一个文件夹，右键点击空白处

   <img src="Assets/1 (1).png" alt="1 (1)" style="zoom:65%;" />

2. 然后选择点击git bash here打开命令行窗口

   <img src="Assets/1 (2).png" alt="1 (2)" style="zoom:50%;" />

3. 复制以下命令

		git clone https://jihulab.com/awakelion-robot-lab/control-group/standard-robot.git

4. 注意不要使用Ctrl+V粘贴，否则会出现错误
   正确粘贴快捷键是Shift+Ins,或者右键点击再选Paste

5. 按Enter执行即可拉取代码

   

#### git命令使用说明
1. git clone 拉取远程仓库的代码到本地，如：

      	git clone https://jihulab.com/awakelion-robot-lab/control-group/standard-robot.git
     
2. 切换至新分支xxx（相当于复制了remote的仓库到本地的xxx分支上）

     	git checkout -b xxx 


3. 修改完成后，查看自己对代码做出的改变

     	git diff 

5. 上传更新后的代码至暂存区

     	git add 

6. 将暂存区里更新后的代码更新到本地git
	

     	git commit -m "xxx"            xxx为你的修改说明,如git commit -m "修改底盘功率算法"

7.  将本地的git上的xxx分支上传至gitlab上

     	git push origin xxx



#### git开发流程
1. 从GitLab拉取代码

    	git clone https://jihulab.com/awakelion-robot-lab/control-group/standard-robot.git

2. 新建my-feature分支，并由main分支切换到my-feature分支，执行以下这一条命令即可          (通常我们把开发分支命名为develop)

			git checkout -b my-feature

3. 在自己电脑上用Keil,Vscode或Qt修改代码或者添加文件

4. 查看对比自己修改的代码

			git diff

5. 添加修改后的版本到本地的git的暂存区
	​		情况a.	针对单个特定文件添加

				git add <changed_file>  如 git add gimbal_task.c

	​		情况b.	修改了多个文件（首先要删除编译文件，电控组的点击kill.bat删除编译出的二进制文件）然后再执行
	
				git add .

6. 将暂存区的修改记录提交更新到本地的git

			git commit -m "xxx"

​		（此时的更新是更新到了my-feature分支，因为在执行了第二步后，我们就一直在my-feature分支了）

7. 将本地的my-feature分支更新到GitLab远程仓库

			git push orgin my-feature

​	以上就是使用git来管理和维护代码的基本工作流程。

8. 版本回退
​	方法一	reset	该命令会强行覆盖当前版本和要回退的版本之间的其他版本（不太建议）
```
   git log                      （查看版本号）
   git reset --hard +目标版本号     （回退到目标版本）
```

​	方法二	revert	在当前版本的基础上新增一个版本，不影响以前的代码
```
   git log                      （查看版本号）
   git revert -n +目标版本号     （回退到目标版本）
```
​	以上就是常用到的全部命令，当然也可以通过vscode+git插件通过图形化操作界面完成。



#### 可能会遇到的情况

​	假设你两天前拉了一份main分支的代码，并在此时此刻你完成了代码的修改，但是在10分钟前GitLab远程仓库的main分支被你的组长更新了，这种情况的解决方法如下：

1. 在你的电脑上由当前的my-feature开发分支切换到main原始主分支

			git checkout main

2. 拉取GitLab远程仓库的最新的main分支代码

			git pull origin main

​		此时你本地的main分支的git和源代码就和GitLab远程仓库的最新的main分支就完全一样了

3. 切换回my-feature开发分支

			git checkout my-feature

​    	此时你的代码又变回了在两天前main基础上修改完成后的样子。

4. 在最新拉下来的mian分支的基础上，自动合并增加你在原来my-feature上的修改生成一个修改后的新的my-feature,请执行以下命令

			git rebase main

​		此时如果一切顺利，相当于你在最新的mian代码上做了你的修改

​		假设出现了rebase conflict，则你需要手动选择你到底要那段代码。

5. 将本地的my-feature分支更新到GitLab远程仓库

			git push orgin my-feature





 



















