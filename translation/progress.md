# 翻訳進捗

MuJoCo日本語ドキュメントの翻訳進捗状況を管理します。

## 凡例

- ❌ 未着手
- ⚠️ 作業中
- ✅ 翻訳完了

## 翻訳状況

### トップレベルページ

| ファイル | 状態 | 備考 |
|---------|------|------|
| index.rst | ⚠️ | 基本構造のみ作成 |
| overview.rst | ❌ | |
| modeling.rst | ❌ | |
| XMLreference.rst | ❌ | 大規模ファイル |
| python.rst | ❌ | |
| unity.rst | ❌ | |
| models.rst | ❌ | |
| changelog.rst | ❌ | 定期的な更新が必要 |

### computation/

| ファイル | 状態 | 備考 |
|---------|------|------|
| index.rst | ❌ | |

### programming/

| ファイル | 状態 | 備考 |
|---------|------|------|
| index.rst | ❌ | |

### APIreference/

| ファイル | 状態 | 備考 |
|---------|------|------|
| index.rst | ❌ | |

### mjx/

| ファイル | 状態 | 備考 |
|---------|------|------|
| （未調査） | ❌ | MuJoCo XLA関連 |

### mjwarp/

| ファイル | 状態 | 備考 |
|---------|------|------|
| index.rst | ❌ | MuJoCo Warp関連 |

### OpenUSD/

| ファイル | 状態 | 備考 |
|---------|------|------|
| index.rst | ❌ | |

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
3. mjx/ - MuJoCo XLA
4. mjwarp/ - MuJoCo Warp

### Phase 4: その他

1. unity.rst - Unity統合
2. OpenUSD/ - OpenUSD統合
3. models.rst - サンプルモデル
4. changelog.rst - 変更履歴

## 次のアクション

- [ ] オリジナルのドキュメント構造を詳細に調査
- [ ] overview.rstの翻訳開始
- [ ] ビルドスクリプトの整備
- [ ] CI/CDの設定
