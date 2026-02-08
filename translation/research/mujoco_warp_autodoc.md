# mujoco_warp autodocエラーの調査

## 問題

ビルド時に以下の警告が出る:

```
WARNING: autodoc: failed to import module 'mujoco_warp'; the following exception was raised:
No module named 'mujoco_warp'
```

## 原因

`mujoco_warp` はPyPIで配布されているパッケージではなく、MuJoCoリポジトリの `mjx/mujoco/mjx/third_party/mujoco_warp/` に埋め込まれたローカルパッケージ。

オリジナルのReadTheDocs上のビルドでは、以下の特別な手順でインストールしている（`.readthedocs.yml` より）:

1. `sed` でimportパスを書き換え（`mujoco.mjx.third_party.mujoco_warp` → `mujoco_warp`）
2. `doc/mjwarp/update_types.py` で型アノテーションをSphinx互換に変換
3. 修正した `mujoco_warp` を `pip install` でインストール

翻訳プロジェクトの仮想環境にはこのモジュールがインストールされていないため、`automodule` ディレクティブが失敗する。

## オリジナルとの設定差分

### conf.py の sys.path

**オリジナル** (`original/mujoco/doc/conf.py`):
```python
sys.path.insert(0, os.path.abspath('../'))
sys.path.append(os.path.abspath('ext'))
sys.path.insert(0, os.path.abspath('../mjx/mujoco/mjx/third_party'))
```

**翻訳プロジェクト** (`docs/conf.py`):
```python
sys.path.append(os.path.abspath('ext'))
```

### .readthedocs.yml のビルド手順（オリジナルのみ）

```yaml
build:
  jobs:
    create_environment:
      # mujoco_warp のimportパスを書き換え
      - |
        find mjx/mujoco/mjx/third_party/mujoco_warp -type f \
          -exec sed -i 's/mujoco\.mjx\.third_party\.mujoco_warp/mujoco_warp/g' {} \;
      # 型アノテーションをSphinx互換に変換
      - python doc/mjwarp/update_types.py mjx/mujoco/mjx/third_party/mujoco_warp/_src/types.py
      # 修正したmujoco_warpをインストール
      - uv pip install mjx/mujoco/mjx/third_party/mujoco_warp
```

### autodoc が参照するファイル

`docs/mjwarp/api.rst` で以下のように使用:
```rst
.. automodule:: mujoco_warp
  :members:
  :imported-members:
```

## 解決策の選択肢

### A. autodoc_mock_imports を使用（推奨）

`docs/conf.py` に以下を追加:
```python
autodoc_mock_imports = ['mujoco_warp']
```

- 警告が消える
- `automodule` ディレクティブは空の出力になる
- 翻訳プロジェクトでは `automodule` が生成する内容は英語docstringなので、モックにしてRSTに翻訳内容を直接書く方が適切

### B. 埋め込みソースをインストール

オリジナルと同じ手順で `mujoco_warp` をインストールする:

```bash
# importパスの書き換え
find original/mujoco/mjx/mujoco/mjx/third_party/mujoco_warp -type f \
  -exec sed -i '' 's/mujoco\.mjx\.third_party\.mujoco_warp/mujoco_warp/g' {} \;

# 型アノテーション変換
python original/mujoco/doc/mjwarp/update_types.py \
  original/mujoco/mjx/mujoco/mjx/third_party/mujoco_warp/_src/types.py

# インストール
uv pip install original/mujoco/mjx/mujoco/mjx/third_party/mujoco_warp
```

- `automodule` が実際のAPI文書を生成する（ただし英語）
- サブモジュール更新のたびに再実行が必要
- Warpランタイムの依存関係で問題が出る可能性あり

### C. suppress_warnings を使用

`docs/conf.py` に以下を追加:
```python
suppress_warnings = ['autodoc']
```

- 警告は消えるが、他のautodoc警告もすべて抑制されてしまう
- 根本的な解決にはならない

## 結論

翻訳プロジェクトとしては **選択肢A（autodoc_mock_imports）** が最も現実的。`automodule` は英語docstringを引っ張るだけなので、翻訳内容をRSTに直接書く運用と合わせるのが自然。
