def parse(text):
    data = {}
    for entry in text.split(","):
        l,r = entry.split(":")
        if l not in data:
            data[l] = []
        data[l].append(r)
    return data
