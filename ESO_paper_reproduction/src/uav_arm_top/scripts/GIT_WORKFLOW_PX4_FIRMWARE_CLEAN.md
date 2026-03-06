# `PX4_Firmware_clean` 后续使用教程

## 0. VS Code 里从改代码到提交推送

这一节专门讲：**在 VS Code 里，怎么从“开始改代码”一路做到“提交并推送到 GitHub”**。

以后默认都在这个仓库里操作：

```bash
/home/cf/PX4_Firmware_clean
```

### 0.1 第一步：在 VS Code 里打开正确的仓库

不要打开旧仓库：

- `/home/cf/PX4_Firmware`
- `/home/cf/PX4_Firmware_backup_20260306_204306`

应该打开这个目录：

```bash
/home/cf/PX4_Firmware_clean
```

在 VS Code 里操作：

1. `File`
2. `Open Folder`
3. 选择：

```bash
/home/cf/PX4_Firmware_clean
```

打开后先看左下角分支名。

你应该能看到类似：

```bash
private/main
```

### 0.2 第二步：开始新任务前先切到主分支并更新

如果你准备开始一个新任务，不要直接在旧分支上改。

先打开 VS Code 终端，执行：

```bash
cd /home/cf/PX4_Firmware_clean
git checkout private/main
git pull --ff-only private private/main
```

这一步的作用是：

- 回到主分支
- 确保本地代码和 GitHub 上最新代码一致

### 0.3 第三步：创建一个新分支

在 VS Code 左下角点击当前分支名，例如：

```bash
private/main
```

然后：

1. 选择 `Create new branch`
2. 输入新分支名字

例如：

```bash
feat/uam-v5-hardware-debug
```

创建后，左下角分支名会变成这个新分支。

以后这次任务的所有改动都在这个分支上完成。

### 0.4 第四步：开始改代码

正常编辑代码即可。

改完之后，打开左侧栏的 **Source Control**：

1. 点击左侧分支图标或源代码管理图标
2. 你会看到本次修改过的文件列表

此时你可以做两件事：

- 点某个文件，查看 diff
- 确认自己到底改了哪些文件

建议每次提交前都先看一眼 diff，不要盲目提交。

### 0.5 第五步：暂存文件

在 **Source Control** 里：

- 单个文件右边的 `+`：暂存这一个文件
- `Changes` 标题右边的 `+`：暂存全部修改

这一步等价于命令行的：

```bash
git add <文件>
```

如果你这次只想提交一部分文件，就只点那些你确认要提交的文件，不要一把全加。

### 0.6 第六步：提交

在 **Source Control** 顶部有一个输入框。

在里面写提交说明，例如：

```bash
Fix uam_v5 offboard arm-state handling
```

然后点击：

- 输入框上方或旁边的 `Commit`
- 或者对勾图标

这一步等价于：

```bash
git commit -m "Fix uam_v5 offboard arm-state handling"
```

### 0.7 第七步：第一次把新分支推到 GitHub

如果这个分支是第一次推送，建议在 VS Code 终端里执行，最稳：

```bash
cd /home/cf/PX4_Firmware_clean
git push -u private feat/uam-v5-hardware-debug
```

注意：

- `feat/uam-v5-hardware-debug` 要替换成你自己的分支名

第一次推送之后，这个分支就和 GitHub 上对应分支绑定了。

### 0.8 第八步：后续继续推送

第一次推送成功后，后面同一个分支继续开发时就简单了。

你可以在 VS Code 里：

- 点击 `Sync Changes`
- 点击 `Push`

也可以直接在终端里：

```bash
git push
```

### 0.9 第九步：如何看自己当前在哪个分支

在 VS Code 里最直接的位置是：

- 左下角状态栏

那里会显示当前分支名。

例如：

```bash
private/main
```

或者：

```bash
feat/uam-v5-hardware-debug
```

如果你不确定，也可以在终端里执行：

```bash
git branch --show-current
```

### 0.10 第十步：如何查看所有分支

在 VS Code 里：

1. 点击左下角分支名
2. 会弹出分支列表
3. 可以看到本地分支和可切换目标

在终端里更清楚：

看本地分支：

```bash
git branch
```

看本地分支和它跟踪的远程分支：

```bash
git branch -vv
```

看所有本地 + 远程分支：

```bash
git branch -a
```

### 0.11 第十一步：一个完整例子

假设你要改 `uam_v5_arm_joint_state_bridge.py`。

完整流程如下：

1. 打开 VS Code，并打开目录：

```bash
/home/cf/PX4_Firmware_clean
```

2. 在终端执行：

```bash
git checkout private/main
git pull --ff-only private private/main
git checkout -b feat/bridge-timeout-fix
```

3. 在编辑器里修改：

```bash
ESO_paper_reproduction/src/uav_arm_top/scripts/uam_v5_arm_joint_state_bridge.py
```

4. 打开 `Source Control`，查看 diff
5. 点击该文件右边的 `+` 暂存
6. 输入提交信息：

```bash
Fix bridge timeout handling
```

7. 点击 `Commit`
8. 在终端执行第一次推送：

```bash
git push -u private feat/bridge-timeout-fix
```

9. 后续继续改这个分支时，只需要：

```bash
git push
```

### 0.12 最重要的三个习惯

以后在 VS Code 里用 Git，只要记住这三条：

1. 先确认左下角当前分支是不是你想要的分支
2. 提交前先在 `Source Control` 里看 diff
3. 新任务一律先从 `private/main` 拉一个新分支，不要直接在 `private/main` 上乱改

## 1. 以后应该在哪个仓库里工作

以后只在这个目录里继续开发：

```bash
/home/cf/PX4_Firmware_clean
```

不要再回到下面这些目录里继续改代码：

- `/home/cf/PX4_Firmware`
- `/home/cf/PX4_Firmware_backup_20260306_204306`

这两个目录分别是：

- 旧的脏仓库
- 旧仓库的只读备份

真正干净、已经推到你私有 GitHub 仓库的是：

```bash
/home/cf/PX4_Firmware_clean
```

## 2. 当前仓库的分支关系

当前仓库已经配置好：

- 本地分支：`private/main`
- 远程私有仓库：`private`
- 远程主线分支：`private/private/main`

也就是说，你现在的主工作线是：

```bash
private/main
```

它跟踪的是你 GitHub 私有仓库里的：

```bash
private/private/main
```

## 3. 每次开始工作前怎么做

先进入仓库：

```bash
cd /home/cf/PX4_Firmware_clean
```

回到主分支：

```bash
git checkout private/main
```

拉取你私有仓库里的最新内容：

```bash
git pull --ff-only private private/main
```

这三步就是你每次开始干活前的标准动作。

## 4. 新功能应该怎么开分支

不要长期直接在 `private/main` 上写代码。

正确做法是：

```bash
cd /home/cf/PX4_Firmware_clean
git checkout private/main
git pull --ff-only private private/main
git checkout -b feat/<功能名>
```

例如：

```bash
git checkout -b feat/uam-v5-real-hardware-debug
```

这样你后续所有改动都会在新分支上，不会污染主分支。

## 5. 改完代码之后怎么提交

先看改了什么：

```bash
git status
git diff
```

再把你确认要提交的文件加进去：

```bash
git add <文件或目录>
```

例如：

```bash
git add src/modules/eso_att_control
git add ESO_paper_reproduction/src/uav_arm_top
```

然后提交：

```bash
git commit -m "Fix uam_v5 offboard arm-state handling"
```

## 6. 怎么推到 GitHub

如果这是一个新分支，第一次推送用：

```bash
git push -u private feat/<功能名>
```

例如：

```bash
git push -u private feat/uam-v5-real-hardware-debug
```

以后这个分支再推送，就直接：

```bash
git push
```

## 7. 我应该在哪里查看分支

分支可以看两处：本地终端和 GitHub 网页。

### 7.1 在终端里看本地分支

看当前在哪个分支：

```bash
git branch --show-current
```

看所有本地分支：

```bash
git branch
```

看本地分支和它跟踪的远程分支：

```bash
git branch -vv
```

看所有本地 + 远程分支：

```bash
git branch -a
```

### 7.2 在 GitHub 网页上看分支

进入你的仓库：

```text
https://github.com/chnchenfan/px4-uam-v5-eso
```

然后看页面左上方靠近分支名的位置，通常会显示：

- `private/main`
- 或你当前最近推上去的分支名

点那个分支下拉框，就能看到：

- 已有分支
- 搜索分支
- 切换到别的分支

## 8. 常用检查命令

### 8.1 看当前仓库是否干净

```bash
git status
```

如果看到：

```bash
nothing to commit, working tree clean
```

说明当前仓库是干净的。

### 8.2 看最近提交历史

```bash
git log --oneline --decorate -n 10
```

### 8.3 看远程配置

```bash
git remote -v
```

你现在应该看到类似：

```bash
origin   https://github.com/PX4/PX4-Autopilot.git
private  ssh://git@ssh.github.com:443/chnchenfan/px4-uam-v5-eso.git
```

含义是：

- `origin`：官方 PX4 仓库
- `private`：你自己的私有仓库

## 9. 推荐日常工作流程

### 场景 1：开始一个新功能

```bash
cd /home/cf/PX4_Firmware_clean
git checkout private/main
git pull --ff-only private private/main
git checkout -b feat/<功能名>
```

### 场景 2：开发过程中提交一次

```bash
git status
git add <文件>
git commit -m "..."
```

### 场景 3：把分支推到 GitHub

```bash
git push -u private feat/<功能名>
```

### 场景 4：继续同一个分支开发

```bash
git checkout feat/<功能名>
git status
git add <文件>
git commit -m "..."
git push
```

### 场景 5：回到主分支准备下一个任务

```bash
git checkout private/main
git pull --ff-only private private/main
```

## 10. 当前最重要的原则

后续只记住这三条：

1. 永远在 `/home/cf/PX4_Firmware_clean` 里工作
2. 开新任务先从 `private/main` 拉一个新分支
3. 用 `git status` 随时检查自己当前是不是在正确分支、仓库是不是干净

## 11. 你现在最常用的几条命令

如果只保留最少的一组，记这几条就够了：

```bash
cd /home/cf/PX4_Firmware_clean
git status
git branch -vv
git checkout private/main
git pull --ff-only private private/main
git checkout -b feat/<功能名>
git add <文件>
git commit -m "..."
git push -u private feat/<功能名>
```

## 12. 这个仓库里的 `sitl_gazebo` 子模块怎么处理

这个仓库里有一个关键子模块：

```bash
Tools/sitl_gazebo
```

它现在已经不是官方 `PX4-SITL_gazebo`，而是你自己的私有仓库：

```bash
ssh://git@ssh.github.com:443/chnchenfan/px4-sitl-gazebo-custom.git
```

原因是：

- `uav_arm_v4` 和 `uam_v5` 的 Gazebo 模型在这个子模块里
- 包括：
  - `Tools/sitl_gazebo/models/uav_arm_v4`
  - `Tools/sitl_gazebo/models/uam_v5`
- 如果还指向官方子模块，新机器拉代码时拿不到这两个模型

### 12.1 新机器应该怎么克隆

最稳妥的方式：

```bash
git clone --recursive git@github.com:chnchenfan/px4-uam-v5-eso.git
```

如果已经 clone 了主仓库，但还没有拉子模块：

```bash
cd /home/cf/PX4_Firmware_clean
git submodule sync --recursive
git submodule update --init --recursive
```

### 12.2 怎么看子模块是不是正常

在主仓库里看：

```bash
cd /home/cf/PX4_Firmware_clean
git submodule status
```

只看 `sitl_gazebo`：

```bash
git submodule status Tools/sitl_gazebo
```

进入子模块看当前分支和状态：

```bash
git -C /home/cf/PX4_Firmware_clean/Tools/sitl_gazebo status
git -C /home/cf/PX4_Firmware_clean/Tools/sitl_gazebo branch -vv
git -C /home/cf/PX4_Firmware_clean/Tools/sitl_gazebo remote -v
```

### 12.3 以后如果你改了 Gazebo 模型，正确提交流程是什么

`Tools/sitl_gazebo` 不是普通目录，所以不能只在主仓库里提交一次。

正确顺序是：

1. 先在子模块里提交

```bash
cd /home/cf/PX4_Firmware_clean/Tools/sitl_gazebo
git status
git add <模型文件>
git commit -m "..."
git push
```

2. 再回到主仓库，提交新的子模块指针

```bash
cd /home/cf/PX4_Firmware_clean
git add Tools/sitl_gazebo
git commit -m "Update sitl_gazebo submodule"
git push
```

### 12.4 怎么判断自己是不是忘了提交子模块

如果你在主仓库 `git status` 里看到类似：

```bash
M Tools/sitl_gazebo
```

说明：

- 子模块的 commit 已经变了
- 但主仓库还没提交新的子模块指针

如果你在子模块目录里看到：

```bash
?? models/...
 M ...
```

说明：

- 你连子模块自己的提交都还没做

### 12.5 当前这套工程里和子模块有关的关键仓库

- 主仓库：
  - `git@github.com:chnchenfan/px4-uam-v5-eso.git`
- `sitl_gazebo` 子模块仓库：
  - `git@github.com:chnchenfan/px4-sitl-gazebo-custom.git`
