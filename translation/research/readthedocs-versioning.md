# Read the Docs バージョン管理調査

## 概要

MuJoCo 3.5.0 リリースに伴い、翻訳ドキュメントの複数バージョン公開方法を調査した結果。

## Read the Docs のバージョン認識の仕組み

Read the Docs は **Git のブランチとタグの両方** からバージョンを自動認識する。

### 初期状態

- プロジェクト追加時、リポジトリ内のすべてのブランチ・タグが検出されるが、初期状態では **非アクティブかつ非表示**
- デフォルトブランチ（`main`）を指す `latest` バージョンだけが自動的にアクティブになる

### 自動生成される特殊バージョン

| バージョン | 生成条件 | 追跡対象 |
|-----------|---------|---------|
| `latest` | 常に作成される | デフォルトブランチ（`main` 等） |
| `stable` | SemVer 準拠のタグ/ブランチが1つでもある場合 | 最新の安定版リリース（プレリリース除外） |

### セマンティックバージョニング（PEP 440 準拠）

- `1.4.2` のような通常バージョンに対応
- `2.0a1` のようなプレリリースも認識するが、`stable` からは除外
- `v` プレフィックス（`v1.4.2`、`v2.0a1`）も許可
- タグが1つでもある場合、`stable` の選択ではタグがブランチより優先される
- `stable` という名前のタグまたはブランチを作成すると、そちらが優先される

## `.readthedocs.yaml` とバージョンの関係

- `.readthedocs.yaml` は Git に格納されているため、**ブランチ/タグごとに異なる設定を持てる**
- バージョンの有効化/無効化やデフォルトバージョンの設定は `.readthedocs.yaml` ではなく、**Web ダッシュボード** または **Automation Rules** で行う

## Flyout メニュー（バージョン切り替え UI）

Read the Docs は Flyout メニューをすべてのドキュメントページに自動挿入する。

### 構成要素

1. バージョン切り替え -- アクティブな全バージョンを一覧表示
2. 言語切り替え -- 翻訳プロジェクトが登録されている場合
3. オフラインフォーマット -- HTML/PDF ダウンロードリンク

### カスタマイズ

- 位置: Settings > Addons > Flyout Menu（Bottom right がデフォルト）
- ソート順: SemVer（デフォルト）、CalVer、アルファベット順、カスタムパターン
- `latest` と `stable` を先頭にソートするかどうか設定可能

## デフォルトバージョンの設定

ユーザーがルート URL にアクセスすると、デフォルトバージョンにリダイレクトされる。

### 設定方法

**方法 A: Web ダッシュボードから手動設定**

1. Admin > Settings > Default version で選択
2. 初期値は `latest`

**方法 B: Automation Rules で自動設定**

- 特定パターンにマッチするバージョン作成時に自動的にデフォルトに設定するルールを作れる

## Automation Rules

新しいブランチやタグが検出された際の処理を自動化する機能。ダッシュボードから設定する。

### マッチング方法

| 方式 | 説明 | 例 |
|------|------|-----|
| Any version | すべての新バージョン | -- |
| SemVer versions | セマンティックバージョニング準拠 | `1.0.0`, `v2.1.3` |
| Custom match | Python 正規表現 | `^1\.\d+$` |

### 利用可能なアクション

- Activate version -- バージョンを有効化してビルド実行
- Hide version -- バージョンを非表示にする
- Make version public / private
- Set version as default -- デフォルトバージョンに設定
- Delete version

## このプロジェクトでの運用方針

### タグ戦略

オリジナル MuJoCo のリリースバージョンに対応するタグを翻訳リポジトリにも作成し、サブモジュールを該当バージョンに固定する。

```
翻訳リポジトリのタグ    サブモジュール（original/mujoco）
3.4.1-dev             → 7418d86e（3.4.0 + 220 commits）
3.5.0                 → 3.5.0 タグ
```

### 手順

1. 現在の状態に `3.4.1-dev` タグを打つ（これまでの翻訳を保存）
2. サブモジュールを 3.5.0 に更新
3. 翻訳の差分を反映（変更のあった10ファイルを更新）
4. 完了後に `3.5.0` タグを打つ
5. `git push origin main --tags`

### Read the Docs ダッシュボード設定

1. Automation Rules で SemVer タグの自動アクティブ化ルールを追加
   - Match: SemVer versions
   - Version type: Tag
   - Action: Activate version
2. Settings > Default version を `stable` に変更

### 結果の URL 構造

| URL | 内容 |
|-----|------|
| `mujoco-docs-ja.readthedocs.io/ja/stable/` | 最新タグ（3.5.0） |
| `mujoco-docs-ja.readthedocs.io/ja/latest/` | main ブランチ最新 |
| `mujoco-docs-ja.readthedocs.io/ja/3.4.1-dev/` | 旧版 |

## 3.5.0 で変更のあるドキュメントファイル

現在のサブモジュール（`7418d86e`）から `3.5.0` タグまでに変更された doc/ 配下のファイル（22コミット、15ファイル）。

### 翻訳済みで更新が必要（10ファイル）

| ファイル | 主な変更内容 |
|---------|------------|
| `modeling.rst` | delay 関連の追記、margin/gap 修正 |
| `programming/index.rst` | リソース関数の公開 |
| `programming/simulation.rst` | delay 関連の追記 |
| `XMLreference.rst` | delay 属性追加等 |
| `unity.rst` | 更新あり |
| `APIreference/APIfunctions.rst` | 関数追加 |
| `APIreference/APIglobals.rst` | グローバル変数追加 |
| `APIreference/functions.rst` | 関数追加 |
| `mjwarp/index.rst` | オプション更新、beta 警告削除 |
| `changelog.rst` | 3.5.0 リリースノート |

### 翻訳不要だが同期が必要

- `XMLschema.rst` -- XML スキーマ定義
- `functions_override.rst` -- 自動生成の入力ファイル
- `includes/references.h` -- ヘッダーファイル
- `images/modeling/delay_buffer_dark.svg` -- 画像
- `images/modeling/delay_buffer_light.svg` -- 画像

## conf.py の更新（推奨）

バージョン番号をドキュメントに表示するため、`conf.py` に以下を追加するとよい。

```python
version = '3.5.0'
release = '3.5.0'
```

## 参考資料

- [Versions -- Read the Docs user documentation](https://docs.readthedocs.com/platform/latest/versions.html)
- [Flyout menu -- Read the Docs user documentation](https://docs.readthedocs.com/platform/en/stable/flyout-menu.html)
- [Automation rules -- Read the Docs user documentation](https://docs.readthedocs.com/platform/stable/automation-rules.html)
- [Configuration file reference -- Read the Docs user documentation](https://docs.readthedocs.com/platform/stable/config-file/v2.html)
