"""v6: the page's "What it costs" tables, from prices.json (bought parts, read from each vendor page) and scs_quote.json
(SendCutSend instant quote). Written into viewer.html between <!--COST_TABLE--> markers.

    python cost_page.py
"""
import json
import re
from pathlib import Path

HERE = Path(__file__).parent


def money(x):
    return f"${x:,.2f}"


def main_v7(p):
    """v7: no cut aluminum. Plywood sheets and PA-CF filament are bought items in prices.json like everything else."""
    import turret as T
    tot = p["totals"]
    bought = tot["grand_total_usd"]
    h = [f'<p class="lede">Prices checked {p["date_checked"]}, before shipping and tax. The frame is cut from plywood on the '
         'team\'s laser and most other parts are printed, so the only cost for those is the material.</p>',
         '<table class="adj cost"><tr><th>What</th><th>From</th><th>Cost</th></tr>']
    for v, amt in tot["by_vendor"].items():
        h.append(f"<tr><td>Parts and materials</td><td>{v}</td><td>{money(amt)}</td></tr>")
    h.append(f'<tr><th>Total</th><th></th><th>{money(bought)}</th></tr></table>')
    ll = next((i for i in p["items"] if i["sku"] == "3122-0002-0001"), None)
    h.append("<p>Not counted: the Limelight 3A camera (the team already owns one; "
             + (f"{money(ll['price_usd'])} if you need another" if ll else "about $189 new")
             + "), and the PETG for the printed parts (crossbars, L-brackets, rear guard, camera mount, switch arm and so on: "
             "about 1 kg, about $20). The PA-CF spool covers the parts printed in carbon-fiber nylon: the swing "
             "arms, pulleys, spacers, servo insert and motor adapter." + ("" if T.CABLES else " Cable management isn't counted yet: "
             "it is chosen after a bench test (see the design log).") + "</p>")
    h.append('<details><summary>Every part and material</summary><table class="adj"><tr><th>Part</th><th>Buy</th><th>Price</th><th>Line</th></tr>')
    for i in p["items"]:
        h.append(f'<tr><td><a href="{i["url"]}" target="_blank" rel="noopener">{i["sku"]}</a> {i["name"]}</td>'
                 f'<td>{i["qty_units_to_buy"]} × {i["unit_desc"]}</td><td>{money(i["price_usd"])}</td><td>{money(i["line_total_usd"])}</td></tr>')
    h.append("</table></details>")
    return h, f"cost: parts and materials {money(bought)}"


def main():
    p = json.loads((HERE / "prices.json").read_text())
    import turret as T
    if T.LAMINATE:
        h, msg = main_v7(p)
        v = HERE / "viewer.html"
        src = v.read_text()
        new = re.sub(r"<!--COST_TABLE-->.*?<!--/COST_TABLE-->", lambda _: "<!--COST_TABLE-->" + "\n".join(h) + "<!--/COST_TABLE-->", src, flags=re.S)
        v.write_text(new); print(msg)
        return
    q = json.loads((HERE / "scs_quote.json").read_text())
    tot = p["totals"]
    bought = tot["grand_total_usd"]
    ll = next((i["line_total_usd"] for i in p["items"] if i["sku"] == "3122-0002-0001"), 0.0)
    h = [f'<p class="lede">Prices checked {p["date_checked"]} (parts) and {q["date_checked"]} (plates), before shipping and tax.</p>',
         '<table class="adj cost"><tr><th>What</th><th>From</th><th>Cost</th></tr>']
    for v, amt in tot["by_vendor"].items():
        h.append(f"<tr><td>Bought parts</td><td>{v}</td><td>{money(amt)}</td></tr>")
    h.append(f"<tr><td>Aluminum plates, laser cut (6061-T6)</td><td>SendCutSend</td><td>{money(q['total'])}</td></tr>")
    h.append(f'<tr><th>Total</th><th></th><th>{money(bought + q["total"])}</th></tr></table>')
    h.append(f"<p>The Limelight 3A ({money(ll)}) is part of the goBILDA figure and was out of stock at goBILDA and ServoCity; "
             f"without it the total is {money(bought - ll + q['total'])}. The tapped and countersunk holes aren't in the plate price: "
             "SendCutSend's quote only offers them on holes drawn at its own tap-drill sizes, so plan to do them yourself "
             "(an M4 tap, a tap handle and a countersink bit, about $20) or ask SendCutSend when ordering. "
             "The motor-belt test plate is optional and not included.</p>")
    h.append('<details><summary>Every plate</summary><table class="adj"><tr><th>Plate</th><th>Stock</th><th>Qty</th><th>Each</th><th>Line</th></tr>')
    for i in q["items"]:
        est = " (est.)" if "estimate" in i.get("note", "") else ""
        h.append(f"<tr><td>{i['file']}{est}</td><td>{i['stock']}</td><td>{i['qty']}</td><td>{money(i['each'])}</td><td>{money(i['line'])}</td></tr>")
    h.append("</table></details>")
    h.append('<details><summary>Every bought part</summary><table class="adj"><tr><th>Part</th><th>Buy</th><th>Price</th><th>Line</th></tr>')
    for i in p["items"]:
        h.append(f'<tr><td><a href="{i["url"]}" target="_blank" rel="noopener">{i["sku"]}</a> {i["name"]}</td>'
                 f'<td>{i["qty_units_to_buy"]} × {i["unit_desc"]}</td><td>{money(i["price_usd"])}</td><td>{money(i["line_total_usd"])}</td></tr>')
    h.append("</table></details>")
    v = HERE / "viewer.html"
    src = v.read_text()
    new = re.sub(r"<!--COST_TABLE-->.*?<!--/COST_TABLE-->", lambda _: "<!--COST_TABLE-->" + "\n".join(h) + "<!--/COST_TABLE-->", src, flags=re.S)
    if new == src and "<!--COST_TABLE-->" not in src:
        print("marker not found")
    else:
        v.write_text(new)
        print(f"cost: bought {money(bought)} + plates {money(q['total'])} = {money(bought + q['total'])}")


if __name__ == "__main__":
    main()
