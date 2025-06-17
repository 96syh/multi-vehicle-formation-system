# GitHub上传指南

## 步骤1：在GitHub上创建仓库

1. 访问 [GitHub.com](https://github.com)
2. 登录您的账户
3. 点击右上角的 "+" 号，选择 "New repository"
4. 填写仓库信息：
   - **Repository name**: `multi-vehicle-formation-system`
   - **Description**: `A comprehensive multi-vehicle formation control system with Python simulation and ROS2 integration`
   - **Visibility**: Public (推荐) 或 Private
   - **不要勾选** "Add a README file"（我们已经有了）
   - **不要勾选** "Add .gitignore"（我们已经有了）
   - **不要勾选** "Choose a license"（我们已经有了）

## 步骤2：连接本地仓库到GitHub

在终端中运行以下命令（将 `yourusername` 替换为您的GitHub用户名）：

```bash
# 添加远程仓库
git remote add origin https://github.com/yourusername/multi-vehicle-formation-system.git

# 推送到GitHub
git push -u origin main
```

## 步骤3：验证上传

1. 刷新您的GitHub仓库页面
2. 确认所有文件都已上传
3. 检查README.md是否正确显示

## 可选步骤：配置Git用户信息

如果这是第一次使用Git，建议配置用户信息：

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# 修正当前提交的作者信息
git commit --amend --reset-author
```

## 后续更新

当您修改代码后，使用以下命令更新GitHub仓库：

```bash
git add .
git commit -m "描述您的更改"
git push
```

## 完成！

您的项目现在已经在GitHub上了！其他人可以：
- ⭐ Star您的项目
- 🍴 Fork并贡献代码
- 📥 下载和使用您的代码
- 🐛 报告问题和建议

项目地址将是：`https://github.com/yourusername/multi-vehicle-formation-system` 