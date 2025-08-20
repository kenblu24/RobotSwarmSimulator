# split choosing procedure for one dimension and one direction:
# n = number of segments
# i = index of start of current segment
# e = index of closest to the right segment end
# l = i
# r = n - e
# d = min(n - i + e, l * 2, r * 2)



class PolygonBox:
    def __init__(self, poly, points):
        self.poly = poly
        self.points = points
        self.minX = min([p[0] for p in points])
        self.maxX = max([p[0] for p in points])
        self.minY = min([p[1] for p in points])
        self.maxY = max([p[1] for p in points])

class RectDNode:
    def __init__(self, xMax, xMin, yMax, yMin, capacity = 2):
        self.xMax = xMax
        self.xMin = xMin
        self.yMax = yMax
        self.yMin = yMin
        self.capacity = capacity
        self.count = 0
        self.contents: list(PolygonBox) = []
    def subdivide(self):
        xls = sorted([rect.minX for rect in self.contents])
        xrs = sorted([rect.maxX for rect in self.contents])
        yts = sorted([rect.maxY for rect in self.contents])
        ybs = sorted([rect.minY for rect in self.contents])
    def insert(self, point):
        # Check to ensure we're not going to go over capacity.
        if (len(self.points) + 1) > self.capacity:
            # We're over capacity. Subdivide, then insert into the new child.
            self.subdivide()

        if self.ul is not None:
            if self.is_ul(point):
                self.ul.insert(point)
            if self.is_ur(point):
                self.ur.insert(point)
            if self.is_ll(point):
                self.ll.insert(point)
            if self.is_lr(point):
                self.lr.insert(point)

        # There are no child nodes & we're under capacity. Add it to `points`.
        self.points.append(point)
        return True


def findOptimalSplit(starts, ends, forward=True):
    if not forward:
        starts.reverse()
        ends.reverse()
        starts, ends = ends, starts
    
    bestScore = 0
    bestSplit = 0

    n = len(starts)
    e = 0
    for i in range(n):
        while ends[e] < starts[i] if forward else starts[i] < ends[e]:
            e += 1
        
        l = i # number of segments left of split
        r = n - e # number of segments right of split
        d = n - i + e # number of segments entirely left or right of split
        score = min(d, l * 2, r * 2)

        if bestScore < score:
            bestScore = score
            bestSplit = i
    
    return bestSplit, bestScore

