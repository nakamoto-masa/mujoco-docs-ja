# Read the Docs デプロイ調査

## 概要

現在 GitHub Pages でデプロイしているドキュメントを Read the Docs でホスティングする場合の調査結果。

## URL

`https://mujoco-docs-ja.readthedocs.io/`

## セットアップ手順

1. [readthedocs.io](https://readthedocs.io) に GitHub アカウントでサインアップ
2. 「Import a Project」から `mujoco-docs-ja` リポジトリを選択
3. リポジトリのルートに `.readthedocs.yaml` を追加
4. push すると自動でビルド・公開される

## 必要な設定ファイル

リポジトリのルートに `.readthedocs.yaml` を配置する。

`uv` は Read the Docs にプリインストールされていないため、`asdf` 経由でインストールする（オリジナルの MuJoCo プロジェクトも同じ方法を使用）。

```yaml
version: 2

build:
  os: ubuntu-24.04
  tools:
    python: "3.13"
  jobs:
    pre_create_environment:
      - asdf plugin add uv
      - asdf install uv latest
      - asdf global uv latest
    create_environment:
      - uv venv "${READTHEDOCS_VIRTUALENV_PATH}"
    install:
      - UV_PROJECT_ENVIRONMENT="${READTHEDOCS_VIRTUALENV_PATH}" uv sync --frozen

sphinx:
  builder: html
  configuration: docs/conf.py
  fail_on_warning: false

submodules:
  include:
    - original/mujoco
  recursive: true
```

## 設定のポイント

- **`UV_PROJECT_ENVIRONMENT`**: Read the Docs が期待する virtualenv パスにインストールするために必須
- **`--frozen`**: `uv.lock` をそのまま使い、再現性のあるビルドにする
- **`submodules`**: `original/mujoco` サブモジュールのクローンに必要
- **`fail_on_warning: false`**: 翻訳作業中の警告でビルド失敗しないようにする

## GitHub Pages との比較

| 項目 | GitHub Pages | Read the Docs |
|------|-------------|---------------|
| URL | `nakamoto-masa.github.io/mujoco-docs-ja/` | `mujoco-docs-ja.readthedocs.io/` |
| ビルド設定 | GitHub Actions ワークフロー | `.readthedocs.yaml` |
| 自動ビルド | ワークフローで設定が必要 | push 時に自動 |
| 料金 | 無料 | オープンソースなら無料 |

## 注意事項

- URL のスラッグ（`mujoco-docs-ja`）はプロジェクト作成後に変更不可。名前は慎重に選ぶこと
- `mujoco-mjx` などの重い依存パッケージでビルドがタイムアウトする可能性がある。その場合はドキュメント用の依存グループを分離する
