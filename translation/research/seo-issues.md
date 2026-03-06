# SEO課題：「MuJoCo 日本語」で検索上位に表示されない

## 調査日

2026-03-07

## 現状

「MuJoCo 日本語」で検索しても https://mujoco-docs-ja.readthedocs.io が上位に表示されない。

## オリジナルとの比較

オリジナル（`original/mujoco/doc/conf.py`）と日本語版（`docs/conf.py`）のSEO関連設定を比較した結果、**両者ともSEOは一切考慮されていない**ことが判明。以下の項目はオリジナルにも存在しない。

| 設定項目 | オリジナル | 日本語版 |
|---|---|---|
| `html_baseurl` | なし | なし |
| `sphinx_sitemap` 拡張 | なし | なし |
| `.. meta::` description（トップページ） | なし | なし |
| `language` | 未設定 | `'ja'` |
| index → overview リダイレクト | あり | あり |

本家が「MuJoCo」で検索上位に出るのはSEO設定のおかげではなく、GitHubリポジトリ（12k+ stars）からの大量の被リンク、論文・ブログ・チュートリアルからの引用、ドメイン名とブランド名の一致といったドメイン権威性による。

日本語版は被リンクがほぼないため、サイト側でできる対策を行わないと検索エンジンに認識されにくい。

なお、以下は調査の結果、実質的に問題ないと判断した。

- **index → overview のリダイレクト**: `sphinx_reredirects` による `<meta http-equiv="refresh" content="0">` はGoogle公式では永続的リダイレクトとして解釈されるため、SEO上の影響はない
- **重複コンテンツのリスク**: 翻訳コンテンツはGoogle公式では重複とみなさないとしており、`language = 'ja'` 設定により `<html lang="ja">` が出力されるため言語の認識も問題ない

## 課題一覧

### 1. サイトマップが生成されていない（最重要）

Sphinx本体には `sitemap.xml` の生成機能がなく、`sphinx-sitemap` 拡張も未導入のため、サイトマップが存在しない。検索エンジンがページを効率的にクロール・インデックスできない。

**対策:**
```python
# conf.py
html_baseurl = 'https://mujoco-docs-ja.readthedocs.io/ja/latest/'

extensions = [
    ...
    'sphinx_sitemap',
]
```
```bash
uv add sphinx-sitemap
```

### 2. トップページにメタディスクリプションがない

Googleは meta description がなくてもページ本文から内容を把握でき、70%以上のケースで独自にスニペットを生成する。ランキングへの直接的な影響もない。ただし、トップページ（overview.html）は「MuJoCo 日本語」で検索した際のランディングページとなるため、適切なdescriptionを設定しておくとスニペットの質が安定する。

なお、全ページに同一の description を設定する `html_meta` は逆効果（個別ページのスニペットとして不適切になる）。ページごとにユニークな description を書くのがベストプラクティスだが、翻訳プロジェクトではメンテナンスコストが見合わないため、トップページのみに絞る。

**対策:** `overview.rst` の冒頭に `.. meta::` ディレクティブを追加
```rst
.. meta::
   :description: MuJoCo公式ドキュメントの日本語翻訳。物理シミュレーションエンジンMuJoCoのAPIリファレンス、チュートリアル、モデリングガイドを日本語で提供。
```

### 3. Google Search Consoleに未登録

サイトの所有権確認・インデックス状況の把握・サイトマップ送信ができていない可能性。

**対策:**
1. [Google Search Console](https://search.google.com/search-console/) でサイトを登録
2. サイトマップURL（`https://mujoco-docs-ja.readthedocs.io/ja/latest/sitemap.xml`）を送信
3. インデックス状況を確認

## 優先度

1. サイトマップ生成（`sphinx_sitemap` + `html_baseurl`）
2. トップページのメタディスクリプション追加（`overview.rst` に `.. meta::` ディレクティブ）
3. Google Search Console 登録・サイトマップ送信

