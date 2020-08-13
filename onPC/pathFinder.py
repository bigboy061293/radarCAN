import pyvisgraph as vg
polys = [[vg.Point(0.0,1.0), vg.Point(3.0,1.0), vg.Point(1.5,4.0)],
        [vg.Point(4.0,4.0), vg.Point(7.0,4.0), vg.Point(5.5,8.0)]]
g = vg.VisGraph()
g.build(polys)
shortest = g.shortest_path(vg.Point(1.5,0.0), vg.Point(4.0, 6.0))
print shortest
#[Point(1.50, 0.00), Point(3.00, 1.00), Point(4.00, 6.00)]