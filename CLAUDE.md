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
│   ├── images/ → original/mujoco/doc/images/        # シンボリックリンク
│   ├── _static/ → original/mujoco/doc/_static/      # シンボリックリンク
│   ├── templates/ → original/mujoco/doc/templates/  # シンボリックリンク
│   ├── includes/ → original/mujoco/doc/includes/    # シンボリックリンク
│   ├── css/ → original/mujoco/doc/css/              # シンボリックリンク
│   ├── js/ → original/mujoco/doc/js/                # シンボリックリンク
│   └── ext/ → original/mujoco/doc/ext/              # シンボリックリンク
│
├── translation/       # 翻訳管理
│   ├── progress.md    # バージョン対応状況
│   ├── glossary.md    # 用語集（統一表記）
│   ├── sphinx-config.md  # Sphinx設定とオリジナルとの差分
│   ├── research/      # 調査記録
│   └── logs/          # 過去の作業ログ
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

## 翻訳ルール

以下のルールに従って翻訳を行う。

### 基本方針

- RST構文（ディレクティブ、ラベル、コードブロック等）はそのまま維持する
- コード例（C、XML）は翻訳しない。コード内コメントのみ日本語化する
- 技術用語（MuJoCo、MJCF、mjModel、mjData等）は英語のまま残す
- URL・リンク先はそのまま維持する
- セクションタイトルは日本語に翻訳する
- 翻訳時は必ず `translation/glossary.md` の用語集を参照し、訳語を統一すること
- 訳語が複数考えられる用語が出た場合は、翻訳後に用語集へ追記すること

### インラインマークアップの注意点

SphinxのRSTパーサーでは、以下の点に注意すること。

**1. 全角文字の直後にスペースを入れる**

SphinxはASCII空白・句読点のみをマークアップの区切りとして認識する。全角文字（全角句読点、日本語文字等）の直後にバッククォートがあるとマークアップが無視される。

```rst
# NG
読み込めます。`MeshLab <https://example.com>`__

# OK
読み込めます。 `MeshLab <https://example.com>`__
```

**2. Sphinx記法も同様**

`:ref:`, `:doc:`, `:el:`, `:at:` などのSphinx記法も、全角文字の直後だとマークアップとして認識されない。

```rst
# NG
関数は:ref:`mjModel`を返します。

# OK
関数は :ref:`mjModel` を返します。
```

## 翻訳ワークフロー（RSTファイル）

### 1. オリジナルを参照

翻訳するオリジナルファイルを確認する

- オリジナル: `original/mujoco/doc/overview.rst`
- 翻訳先: `docs/overview.rst`

### 2. 用語の統一を確認

`translation/glossary.md` を参照して、技術用語の統一表記を確認

### 3. 翻訳作業

翻訳先ファイルを作成または編集し、上記の「翻訳ルール」に従って翻訳を実施する

### 4. 翻訳後のチェック

翻訳完了後、以下のコマンドで全角文字直後のマークアップをチェックする。

**検出パターン:** `[^ -~]` = ASCII以外の文字（全角文字：ひらがな、カタカナ、漢字、全角句読点・括弧など）

```bash
# バッククォート（`...`, ``...``）のチェック
grep -En '[^ -~]`[^`]+`' docs/<filename>.rst    # シングル
grep -En '[^ -~]``[^`]+``' docs/<filename>.rst  # ダブル

# Sphinxロール（:ref:, :doc: など）のチェック
grep -En '[^ -~]:\w+:`' docs/<filename>.rst
```

該当箇所が見つかった場合、全角文字の直後にスペースを挿入する。

### 5. ビルド

通常ビルド:

```bash
cd docs && uv run sphinx-build . _build/html
```

toctreeを変更した場合（キャッシュクリア）:

```bash
cd docs && uv run sphinx-build -E . _build/html
```

### 6. 確認

ビルドエラーがないか確認し、レンダリングが正しいかブラウザで確認する。

```bash
open _build/html/index.html
```

### 7. 進捗管理を更新

`translation/progress.md` で翻訳状態を更新

## バージョン更新ワークフロー（差分反映）

新しいバージョンのMuJoCoがリリースされた際の翻訳更新手順。

### 1. サブモジュールの差分を確認

```bash
cd original/mujoco
git fetch
git diff <現在のコミット>..<新バージョンタグ> --name-only -- doc/
```

### 2. 変更のあったファイルの差分を確認

```bash
cd original/mujoco
git diff <現在のコミット>..<新バージョンタグ> -- doc/<filename>.rst
```

### 3. 翻訳ファイルに差分を反映

オリジナルの差分を確認しながら、対応する翻訳ファイルに変更を反映する。
翻訳ルールおよびインラインマークアップの注意点は新規翻訳と同様。

### 4. 翻訳後のチェック・ビルド・確認

新規翻訳ワークフローのステップ4〜6と同様。

### 5. 進捗管理を更新

`translation/progress.md` で対応状況を更新。

### 6. リリース前チェック

翻訳の差分反映が完了したら、以下を確認してからタグを打つ。

- `README.md` のバージョン情報・基準コミットを更新
- `pyproject.toml` の `mujoco-mjx` バージョンがサブモジュールと一致
- `translation/progress.md` のタスクがすべて完了
- RTDビルドが成功し、ページ表示に問題がない
- annotatedタグを打ってpush（`git tag -a <version> -m "<message>"`）

## Jupyter Notebookの翻訳

### 翻訳対象

- **Markdownセル**: テキストを翻訳する
- **Codeセル**: コメントのみを翻訳する

### ファイルフォーマット

Jupyter Notebookファイル（.ipynb）はJSON形式で、セルの`source`フィールドは2種類の保存形式がある。

**配列形式（標準）:**
```json
"source": [
    "# タイトル\n",
    "\n",
    "説明文"
]
```

**文字列形式（非標準）:**
```json
"source": "# タイトル\n\n説明文"
```

**オリジナルファイルは配列形式で保存されている。翻訳時もこの形式を維持すること。**

配列形式を維持する理由:
- オリジナルとの一貫性（差分確認が容易）
- Git差分管理（行単位で変更追跡）
- 公式標準（nbformatライブラリのデフォルト）

### 翻訳時の注意点

- ファイル構造を維持（セル数、セルの順序、メタデータを変更しない）
- 配列形式で保存（文字列形式に変換されないよう注意）

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
