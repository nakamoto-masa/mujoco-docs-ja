# 翻訳ファイル取り込み計画

## 実施済み作業

### 1. 構造変更の調査

HarnessSimulation/docs/CLAUDE.mdを確認し、pandoc互換のための主な変更点を特定:

- **テーブル形式**: グリッドテーブル → `list-table` ディレクティブ
- **全角文字直後のスペース**: pandocの制約でSphinx記法の前にスペースを挿入
- **インラインマークアップ**: 行をまたがないように1行に収める

### 2. オリジナルとの比較

overview.rstでオリジナル（英語版）と翻訳版を比較:

- オリジナルはグリッドテーブルを使用
- 翻訳版はlist-tableに変更されている
- 全角文字直後に`:ref:`, `:doc:`などのSphinx記法を使う際、スペースが挿入されている（27箇所）

### 3. 修正パターンの確立

overview.rstで実際に修正を実施し、パターンを確立:

**修正対象:**
- list-tableのグリッドテーブルへの変換のみ

**全角文字直後のスペース:**
当初はpandoc特有の制約と考えていたが、Sphinxでも同様に必要であることが判明したため、削除せず維持する。

#### 修正1: list-table → グリッドテーブル変換

オリジナルのグリッドテーブル構造を参照し、日本語の文字幅に合わせて調整。

**例:**
```rst
# 変更前（list-table）
.. list-table::
   :header-rows: 1
   :widths: auto

   * -
     - 高レベル
     - 低レベル
   * - **ファイル**
     - MJCF/URDF (XML)
     - MJB (バイナリ)

# 変更後（グリッドテーブル）
+----------------+---------------------------+----------------------------+
|                | 高レベル                  | 低レベル                   |
+================+===========================+============================+
| **ファイル**   | MJCF/URDF (XML)           | MJB (バイナリ)             |
+----------------+---------------------------+----------------------------+
```

### 4. ビルドテスト結果

overview.rstでビルドテスト実施:

- 基本的にビルド成功
- カスタムロール（`:at:`, `:el:`など）が未定義 → 後で対応
- 画像ファイル未配置 → 後で対応
- タイトルアンダーライン長さの警告 → 後で調整

## 事前準備

並列実行前に以下を実施:

1. **必要なディレクトリを作成**
   ```bash
   mkdir -p docs/computation docs/programming
   ```

## 直列実行計画

残り7ファイルを4グループに分けて、サブエージェントで直列処理。
（当初は並列実行を計画したが、Claude Codeの制限により直列実行に変更）

### グループ1: 小規模ファイル（2ファイル）

- `programming/index.rst`（214行）
- `programming/modeledit.rst`（219行）

### グループ2: 中規模ファイル（2ファイル）

- `modeling.rst`（964行）
- `python.rst`（822行）

### グループ3: 大規模ファイル（2ファイル）

- `computation/index.rst`（1,335行）
- `programming/simulation.rst`（839行）

### グループ4: 超大規模ファイル（1ファイル、単独処理）

- `XMLreference.rst`（7,591行）

## 各サブエージェントのタスク（概要）

各グループのサブエージェントが実施する作業の概要:
（具体的な実行コマンドは「実行コマンド」セクションを参照）

1. **ファイルコピー**
   ```bash
   cp ~/repos/HarnessSimulation/docs/mujoco/src/ja/<filename>.rst \
      ~/repos/mujoco-docs-ja/docs/<path>/<filename>.rst
   ```

2. **list-table検出**
   ```bash
   grep -n ".. list-table::" docs/<path>/<filename>.rst
   ```

3. **list-table → グリッドテーブル変換**
   - オリジナルファイル（`original/mujoco/doc/<path>/<filename>.rst`）を参照
   - グリッドテーブルの構造をコピー
   - 日本語テキストに合わせて列幅を調整

4. **全角文字直後のスペースチェック**

   取り込んだファイルで全角文字直後のスペースが正しく挿入されているか確認:
   ```bash
   # Sphinxロール（:ref:, :doc: など）のチェック
   grep -En '[^ -~]:\w+:`' docs/<path>/<filename>.rst
   ```

   該当箇所が見つかった場合、全角文字の直後にスペースを挿入する。

## 実行コマンド

**実行方式**: 以下の4タスクを**直列で**実行（1つずつ順番に実行し、完了を待ってから次へ）

**共通プロンプトテンプレート**:
```
以下のファイルを処理してください:
[ファイルリスト]

各ファイルに対して以下を実施:
1. ~/repos/HarnessSimulation/docs/mujoco/src/ja/ から該当ファイルをコピー
2. list-tableを検出（grep -n ".. list-table::"）
3. オリジナル（original/mujoco/doc/）のグリッドテーブルを参照して変換
4. 全角文字直後のスペースチェック（grep -En '[^ -~]:\w+:`'）

注意: progress.mdは更新しないでください。
```

**注意事項**:
- サンドボックスエラーは無視して良い
- progress.mdは各サブエージェントでは更新しない

---

### タスク1: グループ1 (小規模ファイル2件)

対象ファイル:
- programming/index.rst（214行）
- programming/modeledit.rst（219行）

### タスク2: グループ2 (中規模ファイル2件)

対象ファイル:
- modeling.rst（964行）
- python.rst（822行）

### タスク3: グループ3 (大規模ファイル2件)

対象ファイル:
- computation/index.rst（1,335行）
- programming/simulation.rst（839行）

### タスク4: グループ4 (超大規模ファイル1件)

対象ファイル:
- XMLreference.rst（7,591行）

注意: このファイルは大きいため、慎重に処理してください。

## 後続タスク

全ファイルの取り込み後:

1. **カスタムロールの設定**
   - `includes/roles.rst`をオリジナルからコピー
   - `includes/macros.rst`もコピー
   - `conf.py`の`rst_prolog`を更新

2. **画像ファイルのコピー**
   - `original/mujoco/doc/images/`から必要な画像をコピー

3. **用語集の統合**
   - `~/repos/HarnessSimulation/docs/mujoco/glossary.md` → `translation/glossary.md`

4. **完全ビルドテスト**
   ```bash
   cd docs && rm -rf _build && uv run sphinx-build -b html . _build/html
   ```

## 注意事項

- 各ファイルのlist-table数は事前に確認済み（overview.rstは1個のみ）
- グリッドテーブルの列幅は日本語の文字幅に合わせて手動調整が必要
- タイトルアンダーラインの長さも日本語に合わせて調整が必要な場合がある
- ビルド時の警告は後で一括対応

## 進捗管理

**方針**:
- 各サブエージェントは`progress.md`を**直接更新しない**（競合防止）
- 各サブエージェントは完了報告のみ行う
- メインエージェントが各サブエージェントの完了報告を受け取った時点で、該当ファイルの進捗を更新

**更新対象** (`translation/progress.md`):

- [x] overview.rst（610行）- 完了
- [x] modeling.rst（964行）← グループ2完了時に更新
- [x] XMLreference.rst（7,591行）← グループ4完了時に更新
- [x] python.rst（822行）← グループ2完了時に更新
- [x] computation/index.rst（1,335行）← グループ3完了時に更新
- [x] programming/index.rst（214行）← グループ1完了時に更新
- [x] programming/modeledit.rst（219行）← グループ1完了時に更新
- [x] programming/simulation.rst（839行）← グループ3完了時に更新
