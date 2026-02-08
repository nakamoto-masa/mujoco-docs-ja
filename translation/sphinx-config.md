# Sphinx設定

オリジナル（[original/mujoco/doc/](../original/mujoco/doc/)）との違いと現在の設定状態を記録します。

## 設計方針

1. **オリジナルの踏襲**: 拡張・テーマ・配色はすべてオリジナルと同一にする
2. **差分の最小化**: 翻訳に必要な変更のみ加える
3. **ビルドツール統一**: uvを使用（`uv run sphinx-build`）

## オリジナルとの差分

現在の差分は以下の4点のみ。それ以外の設定（拡張、テーマ、配色、rst_prolog、CSS/JS等）はすべてオリジナルと同一。

### 1. sys.path（削除）

オリジナルではPython API自動生成用のパスを追加しているが、翻訳版では不要なため削除。

```python
# オリジナルのみ（翻訳版では削除）
sys.path.insert(0, os.path.abspath('../'))
sys.path.insert(0, os.path.abspath('../mjx/mujoco/mjx/third_party'))
```

`mujoco_warp` のAPIページ（`docs/mjwarp/api.rst`）は `automodule` の代わりに翻訳準備中のプレースホルダーを表示している。
詳細は [research/mujoco_warp_autodoc.md](research/mujoco_warp_autodoc.md) を参照。

### 2. language（追加）

```python
language = 'ja'
```

### 3. html_title（変更）

```python
# オリジナル
html_title = 'MuJoCo Documentation'

# 翻訳版
html_title = 'MuJoCo ドキュメント 日本語版'
```

### 4. GitHub設定（変更）

```python
# オリジナル
github_username = 'google-deepmind'
github_repository = 'mujoco'

# 翻訳版
github_username = 'nakamoto-masa'
github_repository = 'mujoco-docs-ja'
```

## Makefile

ビルドツールをuvに統一：

```makefile
SPHINXBUILD = uv run sphinx-build
```

## メンテナンス

### オリジナルとの差分確認

```bash
diff original/mujoco/doc/conf.py docs/conf.py
```

**注意**: このファイルは「現在の状態」を記録します。変更履歴はgitコミットログで確認してください。
