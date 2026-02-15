# Claude Code Automation 推奨設定

分析日: 2026-02-16

## 分析ツール

**Claude Code スキル**: `claude-code-setup:claude-automation-recommender`（anthropic公式プラグイン）

このスキルはコードベースのプロジェクト構成・依存関係・ワークフローを分析し、
Claude Code の拡張機能（Hooks, MCP Servers, Skills, Subagents, Plugins）の推奨設定を提示する。

### 分析対象

- `pyproject.toml` — 依存関係・ツールチェーン
- `CLAUDE.md` — プロジェクト規約・翻訳ワークフロー
- `.claude/settings.local.json` — 既存の権限設定
- `docs/` — RST ファイル（33件）、Jupyter Notebook（7件）
- `translation/glossary.md` — 用語集
- `translation/progress.md` — 翻訳進捗

## コードベースプロファイル

- **プロジェクト種別**: Python（Sphinx ドキュメント翻訳プロジェクト）
- **フレームワーク**: Sphinx + ReStructuredText
- **主要ライブラリ**: Sphinx 7.4.7, Furo theme, nbsphinx, mujoco-mjx
- **ツールチェーン**: uv（パッケージ管理）、Ruff（フォーマッタ/リンタ）、just（タスクランナー）
- **コンテンツ規模**: RST 33ファイル、Jupyter Notebook 7ファイル、Git サブモジュールでオリジナル追跡

---

## 推奨設定

### 1. Hooks

#### 1-1. RST マークアップチェック（PostToolUse）

CLAUDE.md に記載されている「全角文字直後のマークアップ」問題は翻訳作業で頻繁に発生する。
RST ファイルを編集するたびに自動チェックすることで、ビルド前にミスを検出できる。

設定場所: `.claude/settings.json`

```json
{
  "hooks": {
    "PostToolUse": [
      {
        "matcher": "Edit|Write",
        "command": "bash -c 'FILE=\"$TOOL_INPUT_FILE_PATH\"; if [[ \"$FILE\" == *.rst ]]; then ISSUES=$(grep -En \"[^ -~]\\`[^\\`]+\\`|[^ -~]:\\w+:\\`\" \"$FILE\" 2>/dev/null); if [ -n \"$ISSUES\" ]; then echo \"WARNING: 全角文字直後にマークアップあり（スペース挿入が必要）:\"; echo \"$ISSUES\"; fi; fi'"
      }
    ]
  }
}
```

#### 1-2. Ruff 自動フォーマット（PostToolUse）

Python ファイル（conf.py、スクリプト等）を編集した際に自動でフォーマットを適用する。

```json
{
  "hooks": {
    "PostToolUse": [
      {
        "matcher": "Edit|Write",
        "command": "bash -c 'FILE=\"$TOOL_INPUT_FILE_PATH\"; if [[ \"$FILE\" == *.py ]]; then uv run ruff format \"$FILE\" && uv run ruff check --fix \"$FILE\"; fi'"
      }
    ]
  }
}
```

---

### 2. MCP Servers

#### context7（ライブドキュメント参照）

Sphinx、ReStructuredText、nbsphinx などのライブラリの最新ドキュメントを即座に参照でき、
翻訳時の RST 構文確認や Sphinx 設定の調整に役立つ。

```bash
claude mcp add context7 -- npx -y @upstash/context7-mcp@latest
```

---

### 3. Skills

#### 3-1. 翻訳スキル（translate-rst）

CLAUDE.md に定義されている翻訳ワークフロー（用語集参照→翻訳→マークアップチェック→ビルド→進捗更新）を
一つのスキルにまとめることで、`/translate-rst overview.rst` のように呼び出せる。

設定場所: `.claude/skills/translate-rst/SKILL.md`

```yaml
---
name: translate-rst
description: MuJoCo RSTファイルの翻訳ワークフローを実行する
---
```

含めるべき内容:
- 用語集の自動参照
- 翻訳ルール（全角文字スペース挿入など）
- マークアップチェックの自動実行
- ビルド & 進捗更新のリマインド

#### 3-2. バージョン差分チェックスキル（check-version-diff）

新バージョンの MuJoCo リリース時にサブモジュールの差分を確認し、
翻訳が必要なファイルを一覧表示するワークフローを自動化する。

設定場所: `.claude/skills/check-version-diff/SKILL.md`

```yaml
---
name: check-version-diff
description: MuJoCoの新バージョンとの差分を確認し、翻訳更新が必要なファイルを特定する
---
```

---

### 4. Subagents

#### 翻訳レビュー（translation-reviewer）

翻訳の品質チェック（用語の一貫性、RST 構文の正確性、原文との対応）を並列で実行できる。
大量のファイルを翻訳した後の一括レビューに有効。

設定場所: `.claude/agents/translation-reviewer.md`

チェック項目:
- 用語集（glossary.md）との整合性
- 全角文字直後のマークアップ問題
- 未翻訳のセクションの検出
- RST 構文の妥当性

---

## 実装優先度

| 優先度 | 項目 | 理由 |
|--------|------|------|
| 高 | RST マークアップチェック Hook | 翻訳作業で最も頻繁に遭遇する問題を自動検出 |
| 高 | translate-rst スキル | 翻訳ワークフロー全体を標準化・効率化 |
| 中 | Ruff 自動フォーマット Hook | Python ファイル編集時の品質維持 |
| 中 | check-version-diff スキル | バージョン更新時の作業効率化 |
| 低 | context7 MCP | Sphinx ドキュメント参照（必要時のみ） |
| 低 | translation-reviewer Subagent | 大量翻訳後のレビュー用 |
