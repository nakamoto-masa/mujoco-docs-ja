# MuJoCo日本語ドキュメント

## プロジェクト概要

MuJoCoの公式ドキュメント（https://mujoco.readthedocs.io）を日本語に翻訳するプロジェクトです。

- **オリジナルリポジトリ**: https://github.com/google-deepmind/mujoco
- **ドキュメント形式**: ReStructuredText (.rst)
- **ビルドツール**: Sphinx
- **ライセンス**: CC BY 4.0

## 技術スタック

- **Python**: 3.13
- **パッケージ管理**: uv
- **ドキュメント**: Sphinx + ReStructuredText
- **バージョン管理**: Git + サブモジュール

## コミット規約

コミットメッセージは [Conventional Commits](https://www.conventionalcommits.org/ja/v1.0.0/) 形式で日本語で記述する。

## 依存関係の管理

### パッケージの追加

**必ず`uv add`コマンドを使用する。pyproject.tomlに直接書かない。**

```bash
# 本番依存
uv add <package>

# 開発依存
uv add --dev <package>
```

## コーディングスタイル

### Python

- **フォーマッター**: Ruff
- **コード変更後は必ず実行**: `uv run ruff format .`
- **リント**: `uv run ruff check .`

## ディレクトリ構成

```
mujoco-docs-ja/
├── README.md          # プロジェクト概要（日英併記）
├── LICENSE            # CC BY 4.0ライセンス
├── pyproject.toml     # uv依存関係管理
├── .python-version    # Python バージョン指定
│
├── original/          # オリジナルリポジトリ
│   └── mujoco/        # Gitサブモジュール
│
├── docs/              # 日本語ドキュメント本体
│   ├── conf.py        # Sphinx設定（日本語対応）
│   ├── docutils.conf  # Docutils設定
│   ├── Makefile       # ビルド用
│   │
│   ├── images/        # 画像
│   ├── _static/       # Sphinx静的ファイル
│   ├── templates/     # Sphinxテンプレート
│   ├── includes/      # 再利用可能なスニペット
│   ├── css/           # カスタムCSS
│   └── js/            # カスタムJavaScript
│
├── translation/       # 翻訳管理
│   ├── progress.md    # 翻訳進捗管理
│   ├── glossary.md    # 用語集（統一表記）
│   └── sphinx-config.md  # Sphinx設定とオリジナルとの差分
│
└── scripts/           # ユーティリティ
```

## 開発環境セットアップ

### 1. リポジトリのクローン

```bash
git clone --recursive https://github.com/nakamoto-masa/mujoco-docs-ja.git
cd mujoco-docs-ja
```

### 2. サブモジュールの初期化（クローン時に--recursiveを忘れた場合）

```bash
git submodule update --init --recursive
```

### 3. Python環境のセットアップ

```bash
uv sync
```

### 4. ドキュメントのビルド

```bash
cd docs
uv run sphinx-build . _build/html
```

### 5. ローカルでプレビュー

```bash
open _build/html/index.html
```

## 翻訳ワークフロー

### 1. オリジナルを参照

```bash
# オリジナルドキュメント
original/mujoco/doc/overview.rst

# 翻訳先
docs/overview.rst
```

### 2. 用語の統一

`translation/glossary.md` を参照して、技術用語の統一表記を確認

### 3. 進捗管理

`translation/progress.md` で翻訳状態を更新

## オリジナルの更新追跡

### サブモジュールの更新

```bash
cd original/mujoco
git fetch
git log HEAD..origin/main  # 新しいコミットを確認
cd ../..
```

### 特定バージョンへの固定

```bash
cd original/mujoco
git checkout 3.2.3  # タグ指定
cd ../..
git add original/mujoco
git commit -m "Pin to MuJoCo 3.2.3"
```

## 主要コマンド

```bash
# 依存関係のインストール
uv sync

# ドキュメントのビルド
cd docs && uv run sphinx-build . _build/html

# クリーンビルド
cd docs && rm -rf _build && uv run sphinx-build . _build/html

# サブモジュール更新
git submodule update --remote original/mujoco
```
