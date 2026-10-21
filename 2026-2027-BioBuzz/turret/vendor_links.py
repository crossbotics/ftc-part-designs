"""Find each purchased part's product page at its vendor and save out/vendor_links.json ({sku: url}).

A page is accepted only if the exact SKU appears on it, so a fuzzy search hit can't sneak in. Results are cached in
vendor_links.json next to this script (commit it); re-run with --refresh to re-check everything.

    python vendor_links.py [--refresh]
"""
import csv
import html
import json
import re
import sys
import time
import urllib.parse
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).parent
CACHE = HERE / "vendor_links.json"
UA = {"User-Agent": "Mozilla/5.0 (Macintosh; Intel Mac OS X 14_0) AppleWebKit/605.1.15 (KHTML, like Gecko) Safari/605.1.15"}


def get(url, tries=3):
    for i in range(tries):
        try:
            req = urllib.request.Request(url, headers=UA)
            with urllib.request.urlopen(req, timeout=30) as r:
                return r.geturl(), r.read().decode("utf-8", "replace")
        except Exception:
            time.sleep(1.5 * (i + 1))
    return url, ""


def page_has_sku(text, sku):
    t = html.unescape(text)
    return re.search(r"(?<![\w-])" + re.escape(sku) + r"(?![\w-])", t, re.I) is not None


def gobilda(sku):
    _, page = get("https://www.gobilda.com/search.php?search_query=" + urllib.parse.quote(sku))
    hrefs = re.findall(r'href="((?:https://www\.gobilda\.com)?/[a-z0-9][a-z0-9-]*/)(?:\?[^"]*)?"', page)
    seen, cands = set(), []
    for h in hrefs:
        u = h if h.startswith("http") else "https://www.gobilda.com" + h
        if u not in seen:
            seen.add(u); cands.append(u)
    prefix = sku.split("-")[0]
    cands.sort(key=lambda u: (prefix not in u, ))          # slugs with the series number first
    for u in cands[:25]:
        final, body = get(u)
        # product pages carry the SKU in the product-view block
        m = re.search(r'data-product-sku[^>]*>\s*([^<\s]+)', body)
        if (m and m.group(1).strip().lower() == sku.lower()) or (m is None and page_has_sku(body, sku) and "productView" in body):
            return final.split("?")[0]
    return None


def andymark(sku):
    _, page = get("https://www.andymark.com/search?q=" + urllib.parse.quote(sku))
    cands = []
    for h in re.findall(r'href="(/products/[^"?#]+)', page):
        u = "https://www.andymark.com" + h
        if u not in cands:
            cands.append(u)
    for u in cands[:10]:
        final, body = get(u)
        if page_has_sku(body, sku):
            return final.split("?")[0]
    return None


def rev(sku):
    u = f"https://www.revrobotics.com/{sku.lower()}/"
    final, body = get(u)
    return final if page_has_sku(body, sku) else None


def main():
    refresh = "--refresh" in sys.argv
    links = {} if refresh or not CACHE.exists() else json.loads(CACHE.read_text())
    rows = [r for r in csv.DictReader(open(HERE / "out" / "bom.csv")) if r["section"] in ("goBILDA", "REV", "AndyMark")]
    todo = [(r["section"], r["sku_or_file"]) for r in rows if r["sku_or_file"] not in links]
    fn = {"goBILDA": gobilda, "REV": rev, "AndyMark": andymark}
    with ThreadPoolExecutor(4) as ex:
        for (vendor, sku), url in zip(todo, ex.map(lambda t: fn[t[0]](t[1]), todo)):
            links[sku] = url
            print(f"{vendor:8s} {sku:18s} {url or 'NOT FOUND'}")
    CACHE.write_text(json.dumps(links, indent=1, sort_keys=True))
    missing = [s for s, u in links.items() if not u]
    print(f"{len(links) - len(missing)} linked, {len(missing)} not found: {missing}")


if __name__ == "__main__":
    main()
