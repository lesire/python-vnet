import filters.filter as f

fp = f.create_filter("pass")
print(fp)
print(fp("src", "tgt"))

fb = f.create_filter("block")
print(fb)
print(fb("src", "tgt"))

fp = f.create_filter("pose", range=10)
print(fp)