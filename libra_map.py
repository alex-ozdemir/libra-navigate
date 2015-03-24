import Queue
import nav_constants

debug_get_dir = False


class LibraMap(object):
    def __init__(self, intersections, halls):
        """
        intersections = ['name1', 'name2', ... ]
        halls = [('namei', 'namej', #width, 'NS'|'EW'), ... ]
        """
        self.vertices = intersections
        self.edges = halls
        self.adj_lists = {key: [] for key in self.vertices}

        self.populate_map()

    def populate_map(self):
        for (v1, v2, width, type_) in self.edges:
            self.adj_lists[v1].append( (v2, width, type_[1]) )
            self.adj_lists[v2].append( (v1, width, type_[0]) )

    def get_neighbors(self, v):
        return [edge[0] for edge in self.adj_lists[v]]

    def has_neighbor_in_dir(self, v, dir_):
        res = dir_ in [edge[2] for edge in self.adj_lists[v]]
        if debug_get_dir: print "      %s" % res
        return res

    def get_dir_and_width(self, v1, v2):
        for (neighbor_v, width, dir_) in self.adj_lists[v1]:
            if neighbor_v == v2:
                return (dir_, width)
        raise Exception('Not adjacent: %s, %s' % (v1, v2))

    def get_intersection_type(self, in_dir, v):
        if debug_get_dir: print "  Getting inter: in %s, vertex %s" % (in_dir, v)
        if debug_get_dir: print "  Neighbors: ", self.get_neighbors(v)
        i_t = []
        for turn_t in [nav_constants.TURN_STRAIGHT,
                           nav_constants.TURN_RIGHT,
                           nav_constants.TURN_LEFT,
                           nav_constants.TURN_AROUND]:
            if debug_get_dir: print "    Checking turn %s" % turn_t#"NESW"[turn_t % 4]
            i_t.append(not self.has_neighbor_in_dir(v, self.get_dest_dir_from_src_dir_and_turn_type(in_dir, turn_t)))
        return i_t

    @staticmethod
    def turn_type(from_dir, to_dir):
        dirs = "NESW"
        i = ( dirs.find(to_dir) - dirs.find(from_dir) ) % 4
        int_to_turn_map = {0: nav_constants.TURN_STRAIGHT, 1: nav_constants.TURN_RIGHT, 2: nav_constants.TURN_AROUND, 3: nav_constants.TURN_LEFT}
        return int_to_turn_map[i]

    @staticmethod
    def get_dest_dir_from_src_dir_and_turn_type(src_dir, turn_t):
        dirs = "NESW"
        res = dirs[(dirs.find(src_dir) - turn_t) % 4]
        if debug_get_dir: print "      Neighbor to the %s?" % res
        return res

    def find_bare_path(self, start, end):
        vertex_q = Queue.PriorityQueue()
        vertex_q.put( (0, [start]) )

        done_v_s = []

        while not vertex_q.empty():
            dist, path = vertex_q.get()
            next_v = path[-1]

            if next_v == end:
                return path

            if next_v in done_v_s:
                continue

            done_v_s.append(next_v)
            for neighbor_v in self.get_neighbors(next_v):
                if neighbor_v not in done_v_s:
                    vertex_q.put( (dist + 1, path + [neighbor_v]) )

        return False

    def get_directions(self, start1, start2, dest):
        if type(dest) == type([]):
            return self._get_directions_list(start1, start2, dest)
        # TODO: check if the start is valid
        path = self.find_bare_path(start2, dest)
        direction = self.get_dir_and_width(start1, start2)[0]
        instructions = []
        for i in xrange(len(path) - 1):
            here = path[i]
            next = path[i + 1]
            (dir_, width) = self.get_dir_and_width(here, next)
            turn_t = self.turn_type(direction, dir_)
            inter_t = self.get_intersection_type(direction, here)
            instructions.append( [inter_t, turn_t, width] )
            direction = dir_
        return instructions

    def _get_directions_list(self, start1, start2, dest):
        if len(dest) == 1:
            return self.get_directions(start1, start2, dest[0])
        else:
            first_dst = dest[0]
            path = self.find_bare_path(start2, dest[0])
            to_first = self.get_directions(start1, start2, dest[0])
            second_last = path[-2]
            assert first_dst != second_last
            return to_first + self._get_directions_list(second_last, first_dst, dest[1:])

def make_libra_small():
    inter = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K']
    halls = [('A', 'K', 7, 'NS'),
             ('B', 'A', 7, 'NS'),
             ('C', 'D', 8, 'NS'),
             ('D', 'H', 8, 'NS'),
             ('E', 'F', 8, 'NS'),
             ('A', 'J', 6, 'EW'),
             ('I', 'B', 8, 'EW'),
             ('B', 'C', 8, 'EW'),
             ('D', 'E', 8, 'EW'),
             ('G', 'F', 8, 'EW')]
    return LibraMap(inter, halls)

def test_libra_small():
    libra = make_libra_small()
    s1, s2, e = 'A', 'B', 'C'
    res = libra.get_directions(s1, s2, e)
    print ('%s%s -> %s:   ' % (s1, s2, e) ), res 
    s1, s2, e = 'A', 'B', 'H'
    res = libra.get_directions(s1, s2, e)
    print ('%s%s -> %s:   ' % (s1, s2, e) ), res 
    s1, s2, e = 'A', 'B', 'F'
    res = libra.get_directions(s1, s2, e)
    print ('%s%s -> %s:   ' % (s1, s2, e) ), res 
    s1, s2, e = 'A', 'B', 'K'
    res = libra.get_directions(s1, s2, e)
    print ('%s%s -> %s:   ' % (s1, s2, e) ), res 
    s1, s2, e, e2, e3 = 'A', 'B', 'K', 'F', 'B'
    res = libra.get_directions(s1, s2, [e, e2, e3])
    print ('%s%s -> %s -> %s -> %s:   ' % (s1, s2, e, e2, e3) ), res 

if __name__ == '__main__':
    test_libra_small()