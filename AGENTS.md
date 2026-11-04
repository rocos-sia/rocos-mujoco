# 项目约定

- 当前版本维持 `3.0.x`；未经用户明确允许，不修改 major 或 minor。
  内部测试仅递增 patch 基线或 RC 编号；兼容性变化须说明，不自动升级 major/minor。
- 创建 Git 提交或维护项目版本时，先读取并应用
  [project-version skill](.agents/skills/project-version/SKILL.md)。每个提交都须更新根目录 `VERSION`。
- 遵循 Google C++ 规范；优先最小范围修改。
