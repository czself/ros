# GitHub 分支整理（2026-09-29）

仓库：`czself/ros`。状态：已完成，GitHub远端由37个分支整理为6个；31个旧备份均已转归档标签并核对提交。默认分支 `main` 保留，不合并或强推任何工作分支。

## 保留的6个分支

| 分支 | 用途 |
|---|---|
| `backup/navigation-person-baseline-20260929` | 更早人物/OCR演示基线 |
| `backup/task009-before-position-margin-20260929` | 上一版连续三轮已通过导航基线 |
| `chore/repo-maintenance-20260926` | 其他原维护工作分支，保留 |
| `codex/navigation-before-rebuild` | 原重构前历史分支，保留 |
| `codex/navigation-photo-run-progress` | 当前开发与最新交接入口 |
| `main` | 原默认分支 |

## 31个旧备份转归档标签

每个标签指向原分支完全相同的提交。只有 GitHub 远端标签核对成功，且原分支提交未被别人更新，才删除该远端备份分支。所有提交与文件通过标签继续可访问。

| 原分支 | 提交 | 归档标签 |
|---|---|---|
| `backup/judge-before-frame-selection-fix-20260928` | `a1379ac` | `archive/branches/backup/judge-before-frame-selection-fix-20260928` |
| `backup/ocr-before-closer-plates-20260928` | `99d42fd` | `archive/branches/backup/ocr-before-closer-plates-20260928` |
| `backup/ocr-hd-camera-20260928` | `0634c71` | `archive/branches/backup/ocr-hd-camera-20260928` |
| `backup/person-refinement-20260928` | `283754c` | `archive/branches/backup/person-refinement-20260928` |
| `backup/recognition-presentation-20260928` | `996e82f` | `archive/branches/backup/recognition-presentation-20260928` |
| `backup/submission-organize-20260928` | `93f3a8a` | `archive/branches/backup/submission-organize-20260928` |
| `backup/task009-before-approach-gain-20260929` | `2cb25d3` | `archive/branches/backup/task009-before-approach-gain-20260929` |
| `backup/task009-before-approach-record-20260929` | `5b5ec57` | `archive/branches/backup/task009-before-approach-record-20260929` |
| `backup/task009-before-command-decision-20260929` | `fe33927` | `archive/branches/backup/task009-before-command-decision-20260929` |
| `backup/task009-before-diagnosis-20260929` | `db6a9b2` | `archive/branches/backup/task009-before-diagnosis-20260929` |
| `backup/task009-before-diagnosis-record-20260929` | `e714a22` | `archive/branches/backup/task009-before-diagnosis-record-20260929` |
| `backup/task009-before-evidence-20260929` | `3a9a237` | `archive/branches/backup/task009-before-evidence-20260929` |
| `backup/task009-before-final-record-20260929` | `1896418` | `archive/branches/backup/task009-before-final-record-20260929` |
| `backup/task009-before-heading-20260929` | `49d2eb2` | `archive/branches/backup/task009-before-heading-20260929` |
| `backup/task009-before-margin-record-20260929` | `0c03309` | `archive/branches/backup/task009-before-margin-record-20260929` |
| `backup/task009-before-person-burst-analysis-20260929` | `10b7ef5` | `archive/branches/backup/task009-before-person-burst-analysis-20260929` |
| `backup/task009-before-phase-metrics-20260929` | `678c722` | `archive/branches/backup/task009-before-phase-metrics-20260929` |
| `backup/task009-before-preferred-pilot-record-20260929` | `f229b02` | `archive/branches/backup/task009-before-preferred-pilot-record-20260929` |
| `backup/task009-before-preferred-record-20260929` | `3af14f6` | `archive/branches/backup/task009-before-preferred-record-20260929` |
| `backup/task009-before-preferred-w-20260929` | `292fce0` | `archive/branches/backup/task009-before-preferred-w-20260929` |
| `backup/task009-before-results-20260929` | `1ef2fcf` | `archive/branches/backup/task009-before-results-20260929` |
| `backup/task009-before-review-20260929` | `0efad7b` | `archive/branches/backup/task009-before-review-20260929` |
| `backup/task009-before-settle-20260929` | `8d14112` | `archive/branches/backup/task009-before-settle-20260929` |
| `backup/task009-before-telemetry-integrity-20260929` | `7318d2d` | `archive/branches/backup/task009-before-telemetry-integrity-20260929` |
| `backup/task009-before-trace-error-isolation-20260929` | `439ae24` | `archive/branches/backup/task009-before-trace-error-isolation-20260929` |
| `backup/task009-before-trace-freeze-20260929` | `d9a61b4` | `archive/branches/backup/task009-before-trace-freeze-20260929` |
| `backup/task009-before-trace-implementation-20260929` | `d0074f9` | `archive/branches/backup/task009-before-trace-implementation-20260929` |
| `backup/task009-before-trace-results-20260929` | `80fa408` | `archive/branches/backup/task009-before-trace-results-20260929` |
| `backup/task009-before-trace-validation-record-20260929` | `ee1672b` | `archive/branches/backup/task009-before-trace-validation-record-20260929` |
| `backup/task009-before-verified-fast-results-20260929` | `80fa408` | `archive/branches/backup/task009-before-verified-fast-results-20260929` |
| `backup/task009-margin-pilot-20260929` | `265b2dc` | `archive/branches/backup/task009-margin-pilot-20260929` |

## 验证版本标签

| 标签 | 提交 | 含义 |
|---|---|---|
| `verified/navigation-fast-20260929` | `10b7ef5` | 当前导航运行源码及三轮完整证据 |
| `verified/navigation-baseline-20260929` | `4a072e1` | 之前三轮通过的导航基线 |
| `verified/ocr-person-baseline-20260929` | `9c0f878` | 更早人物/OCR基线 |

归档映射见 [JSON清单](github_branch_archive_20260929.json)。恢复命令见 [当前交接](HANDOFF_20260929.md)。用户 `/home/sz/game` 工作区及其本地 `codex/navigation-rebuild` 分支不清理；未提交文件不属于远端标签备份。

## 完成核查

- 31个归档标签、3个关键版本标签及整理前文档备份标签均已上传并核对。
- 31个旧备份远端分支已清理；删除时逐个核对原提交，并使用原提交保护的原子操作。
- 6个保留分支已核对，main及其他保留分支提交未变；当前开发分支仅新增交接和整理记录。
- 冻结运行清单43个文件SHA256全部一致。用户原工作区的未提交状态未变。
- 原分支清单与清理后的引用快照在JSON中保存；快照时间为最终文档提交前，当前开发分支后续文档提交会正常前进。
