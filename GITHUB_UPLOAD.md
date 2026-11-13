# 人工提交到 github.com/LP-Forever

**本次只准备文件，没有执行 git init、git add、commit、push，也没有创建远端仓库。** GitHub 的 `LP-Forever` 是账户/组织路径，不是仓库地址。下面以新仓库 `LP-Forever/skill4RSI` 为例；若要放入已有仓库，应使用最后一节，不能直接覆盖其历史。

## 推荐：新建仓库 LP-Forever/skill4RSI

1. 登录 GitHub，打开 https://github.com/new 。Owner 选择 **LP-Forever**，Repository name 填 **skill4RSI**。
2. 建议首次选择 **Private**。不要自动初始化 README、.gitignore 或 License（本地已有前两项；License 尚需确认归属）。
3. 确认你对该 Owner 有创建仓库和 push 权限。配置自己的 Git 作者身份及 GitHub SSH 密钥，或使用 `gh auth login`/安全 credential helper；不要把密码、PAT 写进 URL/配置模板或提交文件。

在**最终确认的 skill4RSI 目录**里执行（路径以实际放置位置为准）：

```bash
cd /path/to/TCD/skill4RSI
# 首次创建；这些步骤尚未由助手执行
# 若此目录已经属于另一个 Git 仓库，先停止并确认仓库根目录。
git init -b main

# 修改成你自己的真实提交身份；仅写本仓库配置
# git config user.name "Your Name"
# git config user.email "Your verified email"

# 先核对忽略项及文件列表，尤其 memory、参考照片、标定、运行结果、身份信息
git status --short --untracked-files=all
git add --dry-run .

# 检查无误后才实际暂存、提交、推送
git add .
git diff --cached --stat
git diff --cached --name-only
# 确认没有任何 real memory、模型输入/响应、视频、密钥或本地配置再执行：
git commit -m "Add current V2 skill integration and initial ROS adapter"
git remote add origin git@github.com:LP-Forever/skill4RSI.git
git remote -v
git push -u origin main
```

如使用 HTTPS，remote 改成 `https://github.com/LP-Forever/skill4RSI.git`，使用 GitHub CLI 或 credential helper 认证；**不是 GitHub 登录密码，不要将 PAT 嵌入 URL**。若 origin 已存在，先核对；不要重复添加或盲目替换。若远端不是空仓库，先拉取并整合；不要 force push。

本地文件夹名和仓库名是两回事：上述新仓库把本地 `skill4RSI/` 的**内容**作为仓库根目录，网页显示 v2/、ros/、PNG 等；不再多套一层 skill4RSI/。

## 如果要在已有仓库中保留 skill4RSI/ 这一层

先确定目标仓库名，例如 `LP-Forever/<EXISTING_REPO>`。在另一个工作目录 clone 它，将整个 skill4RSI 文件夹复制为其子目录；遇到同名目录先比较，不覆盖。然后在该 clone 中只暂存该子目录：

```bash
git clone git@github.com:LP-Forever/<EXISTING_REPO>.git
cd <EXISTING_REPO>
# 复制前检查 skill4RSI/ 是否存在；已存在则先比较，不覆盖。
cp -a /path/to/TCD/skill4RSI ./skill4RSI
# 注意：若本地 skill4RSI 已执行过 git init，复制时必须排除其 .git，避免嵌套仓库。
git add --dry-run skill4RSI/
git add skill4RSI/
git diff --cached --stat
git commit -m "Add skill4RSI integration snapshot"
git push
```

尖括号是必须替换的占位符；不要把这些示例当成已存在的远端。不要在包含 `needtransformer` 的 TCD 父目录执行 `git add .`，也不要把 needtransformer 的 `.git` 或代码意外纳入此次仓库。

## 推送前验收清单

- 根 README 明确 V2 当前使用、ROS 初版；没有把 ROS 初版称为当前默认/已完整验收。
- memory 中仅空 `.gitkeep`；没有 episode/legacy/preferences 或参考照片。
- 配置不包含真实相机身份、个人绝对路径、真实现场标定或秘密。
- 视频、run/session、NPZ、cache、`.local.*` 与 local_site/ 已忽略。
- 外部 V2/ROS/FrankaPy/libfranka/模型桥没有打包；依赖、许可证和来源说明已看过。
- 如果仍需复现实验数据，用独立受控的数据归档；不要把未审查数据批量塞入 Git。
