#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+ OR MIT
#
# Generate the J714c per-machine PMP tunables dtsi from the Mac's ADT.
#
# Every /arm-io/pmp0/iop-pmp0-nub property (except ADT structural metadata)
# becomes apple,tunable-<name> with the raw ADT bytes, per the naming in
# omacom-m1n1 src/kboot_pmp_t8122.c and the drivers/soc/apple/t6030-j516s-pmp.dtso
# precedent. apple,board-id / apple,dram-vendor-id live on the pmp node already;
# apple,dram-capacity / apple,pio-ranges are not carried (as in the precedent).
#
# Usage: gen-t6050-j714c-pmp-tunables.py <j714c.adt> <out.dtsi>
import sys
sys.path.insert(0, "/home/ed/dev/omarchy/forks/omacom-m1n1/proxyclient")
from m1n1.adt import load_adt, build_prop

adt_path, out_path = sys.argv[1], sys.argv[2]
adt = load_adt(open(adt_path, "rb").read())
nub = adt["/arm-io/pmp0/iop-pmp0-nub"]
chosen = adt["/chosen"]

SKIP = {"name", "compatible", "AAPL,phandle", "region-base", "region-size",
        "segment-names", "segment-ranges", "firmware-name"}

def raw(node, name):
    v = node._properties[name]
    if name in node._types:
        t, _ = node._types[name]
        return bytes(build_prop(node._path, name, v, t=t))
    return bytes(v)

def s(node, key):
    v = node._properties.get(key)
    if isinstance(v, str):
        return v
    if isinstance(v, (bytes, bytearray)):
        return bytes(v).split(b"\0")[0].decode("ascii", "replace")
    return str(v)

def hexb(b):
    return " ".join("%02x" % x for x in b)

out = []
out.append("// SPDX-License-Identifier: GPL-2.0+ OR MIT")
out.append("/*")
out.append(" * T6050 J714c per-machine PMP tunables -- GENERATED, do not edit by hand.")
out.append(" *")
out.append(" * Every /arm-io/pmp0/iop-pmp0-nub property (bar ADT structural metadata) as")
out.append(" * apple,tunable-<name>, per omacom-m1n1 src/kboot_pmp_t8122.c and the")
out.append(" * drivers/soc/apple/t6030-j516s-pmp.dtso precedent. Board-specific: include")
out.append(" * only from t6050-j714c.dts, never the SoC dtsi.")
out.append(" *")
out.append(" * Source ADT: %s" % adt_path)
out.append(" *   chip-id %#x, board-id %#x" % (chosen._properties.get("chip-id", 0),
                                               chosen._properties.get("board-id", 0)))
out.append(" *   firmware-version %r" % s(chosen, "firmware-version"))
out.append(" *   system-firmware-version %r" % s(chosen, "system-firmware-version"))
out.append(" *   iboot %r / %r" % (s(chosen, "iboot-stage-one-tag"), s(chosen, "iboot-stage-two-tag")))
out.append(" *   PMP firmware image %r (uuid %r)" % (s(nub, "firmware-name"), s(nub, "uuid")))
out.append(" * Regenerate: gen-t6050-j714c-pmp-tunables.py j714c.adt t6050-j714c-pmp-tunables.dtsi")
out.append(" */")
out.append("")
out.append("&pmp {")
n = 0
for name in nub._properties.keys():
    if name in SKIP:
        continue
    out.append("\tapple,tunable-%s = [%s];" % (name, hexb(raw(nub, name))))
    n += 1
out.append("};")

open(out_path, "w").write("\n".join(out) + "\n")
print("wrote %s: %d tunables (skipped %d structural)" %
      (out_path, n, sum(1 for k in nub._properties if k in SKIP)))
