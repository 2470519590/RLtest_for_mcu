# 平步

# 分支规范
![分支规范](https://img-blog.csdnimg.cn/img_convert/18e377dd3df53852d3a093110f6ed903.png)
|分支名称|分支说明|
|----|----|
|Production 生产分支|即 Master分支。只能从其他分支合并，不能直接修改|
|Release	预发布（测试）分支|基于 Develop 分支创建，待测试完成后合并到 Develop 或 Production 分支去|
|Develop	主开发分支|包含所有要发布到下一个 Release 的代码，该分支主要合并其他分支内容|
|Feature	新功能分支|基于 Develop 分支创建，开发新功能，待开发完毕合并至 Develop 分支|
|Hotfix	修复分支|基于 Production 分支创建，待修复完成后合并到 Develop 或 Production 分支去，同时在Production上打一个标记|

原文链接：https://blog.csdn.net/sunyctf/article/details/130587970
# 提交规范

本规范基于[Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)制定

## Commit Message 格式

`[type]([scope]): [subject]`

## Type 格式(必填)

- docs: 文档(documentation)

- feat: 新功能(feature)

- fix/to 修复bug

  - fix: 产生diff并自动修复此问题。适用于一次提交修复问题

  - to: 只产生diff不自动修复问题。适用于多次提交。最终修复问题使用fix

- style: 格式(不影响代码运行的变动)

- refactor: 重构(不改变代码本身逻辑)

- perf: (优化相关)

- test: (增加测试)

- chore: (构建过程或辅助工具的变动)

- revert: (回滚到上一个版本)

- merge: (代码合并)

- sync: (同步主线或分支的bug)

## Scope 范围(可选)

用于说明commit影响的范围，比如数据层，控制层，逻辑层等等，视项目不同而不同。

## Subject 主体(必填)

用于简短描述commit目的，不超过50个字符。以动词开头，使用第一人称现在时。