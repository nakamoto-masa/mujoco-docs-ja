# 翻訳進捗

MuJoCo日本語ドキュメントの翻訳進捗状況を管理します。

## 凡例

- ❌ 未着手
- ⚠️ 作業中
- 🔄 取り込み予定（HarnessSimulation/docsから）
- ✅ 翻訳完了
- 🈚 翻訳不要

## 翻訳状況

### トップレベルページ

| ファイル | 状態 | 備考 |
|---------|------|------|
| index.rst | ⚠️ | 基本構造のみ作成 |
| overview.rst | ✅ | 610行（取り込み完了） |
| modeling.rst | ✅ | 964行（取り込み完了） |
| XMLreference.rst | ✅ | 7,591行（取り込み完了） |
| XMLschema.rst | ❌ | XMLreference.rstにインクルード（単独ビルド対象外） |
| python.rst | ✅ | 822行（取り込み完了） |
| mjx.rst | ❌ | MuJoCo XLA |
| mjx_api.rst | ❌ | MuJoCo XLA API（目次外） |
| unity.rst | ❌ | Unity統合 |
| models.rst | ❌ | サンプルモデル |
| changelog.rst | ❌ | 定期的な更新が必要 |

### computation/

| ファイル | 状態 | 備考 |
|---------|------|------|
| index.rst | ✅ | 1,335行（取り込み完了） |
| fluid.rst | ❌ | 流体シミュレーション |

### programming/

| ファイル | 状態 | 備考 |
|---------|------|------|
| index.rst | ✅ | 214行（取り込み完了） |
| simulation.rst | ✅ | 839行（取り込み完了） |
| visualization.rst | ❌ | 可視化 |
| ui.rst | ❌ | ユーザーインターフェース |
| modeledit.rst | ✅ | 219行（取り込み完了） |
| samples.rst | ❌ | サンプルコード |
| extension.rst | ❌ | 拡張機能 |

### APIreference/

| ファイル | 状態 | 備考 |
|---------|------|------|
| index.rst | ❌ | |
| APItypes.rst | ❌ | 型定義 |
| APIfunctions.rst | ❌ | 関数リファレンス |
| APIglobals.rst | ❌ | グローバル変数 |
| functions.rst | 🈚 | ヘッダーから自動生成 |
| functions_override.rst | ❌ | インクルード用（単独ビルド対象外） |

### mjwarp/

| ファイル | 状態 | 備考 |
|---------|------|------|
| index.rst | ❌ | MuJoCo Warp概要 |
| api.rst | ❌ | MuJoCo Warp API |

### OpenUSD/

| ファイル | 状態 | 備考 |
|---------|------|------|
| index.rst | ❌ | OpenUSD統合概要 |
| building.rst | ❌ | ビルド方法 |
| mjcPhysics.rst | ❌ | mjcPhysicsスキーマ |
| mjcf_file_format_plugin.rst | ❌ | MJCFファイル形式プラグイン |
| importing.rst | ❌ | インポート |
| exporting.rst | ❌ | エクスポート |

## 優先順位

### Phase 1: 基本ドキュメント（最優先）

1. overview.rst - MuJoCoの概要
2. modeling.rst - モデリング入門
3. programming/index.rst - プログラミング入門

### Phase 2: リファレンス

1. XMLreference.rst - XML仕様
2. APIreference/index.rst - API仕様

### Phase 3: 高度な機能

1. computation/ - 計算詳細
2. python.rst - Python バインディング
3. mjx.rst / mjx_api.rst - MuJoCo XLA
4. mjwarp/ - MuJoCo Warp

### Phase 4: その他

1. unity.rst - Unity統合
2. OpenUSD/ - OpenUSD統合
3. models.rst - サンプルモデル
4. changelog.rst - 変更履歴

## 次のアクション

### 新規翻訳作業

- [x] オリジナルのドキュメント構造を詳細に調査
- [ ] 未翻訳ファイルの翻訳開始
- [x] ビルドスクリプトの整備
- [ ] CI/CDの設定
