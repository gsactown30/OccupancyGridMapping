def bresenhamLow(start, end):
    startX, startY = start
    endX, endY = end
    arr = []
    dx = endX - startX
    dy = endY - startY
    yi = 1
    if dy < 0:
        yi = -1
        dy = -dy
    D = (2 * dy) - dx
    y = startY

    for point in range(startX, endX):
        coord = (point, y)
        arr.append(coord)
        if D > 0:
            y = y + yi
            D = D + (2 * (dy - dx))
        else:
            D = D + 2 * dy

    return arr

def bresenhamHigh(start, end):
    startX, startY = start
    endX, endY = end
    arr = []
    dx = endX - startX
    dy = endY - startY
    xi = 1
    if dx < 0:
        xi = -1
        dx = -dx
    D = (2 * dx) - dy
    x = startX

    for point in range(startY, endY):
        coord = (x, point)
        arr.append(coord)
        if D > 0:
            x = x + xi
            D = D + (2 * (dx - dy))
        else:
            D = D + 2 * dx

    return arr

def bresenham(start, end):
    startX, startY = start
    endX, endY = end
    arr = []
    if abs(endY - startY) < abs(endX - startX):
        if startX > endX:
            arr = bresenhamLow(end, start)
            arr.pop(0)
            arr.append(start)
        else:
            arr = bresenhamLow(start, end)
    else:
        if startY > endY:
            arr = bresenhamHigh(end, start)
            arr.pop(0)
            arr.append(start)
        else:
            arr = bresenhamHigh(start, end)

    return arr