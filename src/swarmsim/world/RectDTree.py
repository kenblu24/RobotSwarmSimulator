# split choosing procedure for one dimension and one direction:
# n = number of segments
# i = index of start of current segment
# e = index of closest to the right segment end
# l = i
# r = n - e
# d = min(n - i + e, l * 2, r * 2)


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

class NTangle:
    def __init__(self, n, minimum, maximum):
        self.n = n
        self.minimum = minimum
        self.maximum = maximum
    def clone(self):
        return NTangle(self.n, [v for v in self.minimum], [v for v in self.maximum])
    def validate(self):
        if len(self.minimum) != self.n or len(self.maximum) != self.n or any([self.maximum[i] < self.minimum[i] for i in range(self.n)]):
            raise ValueError("invalid NTangle")

class PolygonBox(NTangle):
    def __init__(self, poly, points):
        self.poly = poly
        self.points = points
        super().__init__(self, 2, (min([p[0] for p in points]), min([p[1] for p in points])), (max([p[0] for p in points]), max([p[1] for p in points])))

def NTangleIntersect(a: NTangle, b: NTangle):
    for i in range(len(a.minimum)):
        if b.maximum[i] < a.minimum[i] or a.maximum[i] < b.minimum[i]:
            return False
    return True

class RectDNode:
    def __init__(self, boundingBox: NTangle, capacity = 4):
        self.boundingBox = boundingBox
        self.capacity = capacity
        self.count = 0
        self.contents: list(PolygonBox) = []
        self.children: list(RectDNode) = None
    def bestSplit(self):
        xls = sorted([max(rect.minimum[0], self.boundingBox.minimum[0]) for rect in self.contents])
        xrs = sorted([min(rect.maximum[0], self.boundingBox.maximum[0]) for rect in self.contents])
        yts = sorted([max(rect.minimum[1], self.boundingBox.minimum[1]) for rect in self.contents])
        ybs = sorted([min(rect.maximum[1], self.boundingBox.maximum[1]) for rect in self.contents])

        xf = findOptimalSplit(xls, xrs)
        xb = findOptimalSplit(xls, xrs, False)
        yf = findOptimalSplit(yts, ybs)
        yb = findOptimalSplit(yts, ybs, False)

        options = [xf, xb, yf, yb]
        return = max([(*options[i], i) for i in range(len(options))], key=lambda p : p[1])

    def subdivide(self):
        best = self.bestSplit()
        splitDimension = best[2] // 2 # for the x options which occupy indicies 0 and 1, this will be zero, but for the y options it will be 1
        splitByY = splitDimension == 1
        backward = best[2] % 2 # for the forward options which occupy indicies 0 and 2, this will be zero, but for the forward options it will be 1
        splitIdx = best[0] if backward == 0 else len(self.contents) - 1 - best[0] # if forward just take the index, but if backwards reverse the index
        splitVal = [xls, xrs, yts, ybs][2 * splitByY + backward][splitIdx]

        lessBB = self.boundingBox.clone()
        lessBB.maximum[splitDimension] = splitVal

        moreBB = self.boundingBox.clone()
        moreBB.minimum[splitDimension] = splitVal

        less = RectDNode(lessBB)
        more = RectDNode(moreBB)

        lessContents = [rect for rect in self.contents if rect.minimum[splitDimension] < splitVal]
        moreContents = [rect for rect in self.contents if splitVal < rect.maximum[splitDimension]]

        self.contents = []

        less.insert(lessContents)
        more.insert(moreContents)

        self.children = [less, more]
    def insert(self, rects):
        if self.children:
            for child in self.children:
                child.insert([rect for rect in rects if child.intersect(rect)])
            
        else: 
            self.contents.extend(rects)
            if self.bestSplit()[1] > self.capacity:
                self.subdivide()
        return
    def intersect(self, other: NTangle):
        return NTangleIntersect(self.boundingBox, other)



