# Sphinx設定

オリジナル（[original/mujoco/doc/](../original/mujoco/doc/)）との違いと現在の設定状態を記録します。

## 設計方針

### 基本方針

1. **段階的な構築**: 翻訳の進行に合わせて必要な機能を追加
2. **オリジナルの踏襲**: 見た目（Furoテーマ、配色）は維持
3. **日本語対応**: `language = 'ja'` を設定
4. **ビルドツール統一**: uvを使用（`uv run sphinx-build`）
5. **依存関係の最小化**: 必要な拡張のみを含める

## オリジナルとの差分

### 1. conf.py

#### 日本語設定

```python
language = "ja"
html_title = "MuJoCo ドキュメント 日本語版"
```

#### 使用中のSphinx拡張

```python
extensions = [
    "sphinx.ext.extlinks",  # GitHubリンク（issue/PR）
    "sphinx_copybutton",  # コードブロックコピーボタン
    "sphinx_design",  # デザインコンポーネント
]
```

**オリジナルから除外している拡張:**
- Python API自動生成関連: `autodoc`, `napoleon`, `viewcode`
- 高度な機能: `katex`, `bibtex`, `youtube`, `favicon`, `sphinx_toolbox.*`
- カスタム拡張: `mujoco_include`

#### その他の主な設定

```python
# テーマ
html_theme = "furo"
html_theme_options = {...}  # オリジナルと同じ配色

# RSTプロローグ（簡略版）
rst_prolog = """
.. include:: <isonum.txt>
"""

# GitHubリンク用
extlinks = {
    "issue": ("https://github.com/google-deepmind/mujoco/issues/%s", "issue #%s"),
    "pr": ("https://github.com/google-deepmind/mujoco/pull/%s", "PR #%s"),
}
```

**オリジナルから除外している設定:**
- `sys.path`の追加（Python API生成用パス）
- `redirects`辞書（ページリダイレクト）
- `html_css_files`/`html_js_files`（カスタムCSS/JS）
- `setup()`関数（Sphinx警告フィルター）

### 2. docutils.conf

オリジナルと同じ設定を使用：

```
[html writers]
table-style: colwidths-grid
```

### 3. Makefile

ビルドツールをuvに統一：

```makefile
SPHINXBUILD = uv run sphinx-build
```

## メンテナンス

### 拡張機能を追加する場合

1. `pyproject.toml`に依存関係を追加（必要な場合）
2. `docs/conf.py`の`extensions`に追加
3. 拡張固有の設定を追加
4. このファイルの「使用中のSphinx拡張」セクションを更新
5. ビルドテスト: `cd docs && uv run sphinx-build . _build/html`

### オリジナルとの差分確認

```bash
# conf.pyの差分
diff original/mujoco/doc/conf.py docs/conf.py

# オリジナルの設定を参照
cat original/mujoco/doc/conf.py
```

**注意**: このファイルは「現在の状態」を記録します。変更履歴はgitコミットログで確認してください。
