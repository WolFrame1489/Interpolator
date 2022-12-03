from gcodeparser import GcodeParser
weight = 0
MoveList = []
class Movement():
    def __init__(self):
        self.type = ''
        self.speed = 0
        self.MaxSpeed = 0
        self.MaxAcc = 0
        self.sx = 0
        self.ex = 0
        self.sy = 0
        self.ey = 0
        self.sz = 0
        self.ez = 0
def HandleGCode(filename, pos):
    global weight
    sx = pos[0]
    sy = pos[1]
    sz = pos[2]

    # open gcode file and store contents as variable
    with open(filename, 'r') as f:
        gcode = f.read()
    fw = open('testtraj.cpt', 'w')
    parsed_gcode = GcodeParser(gcode)
    i = 0
    #print(parsed_gcode.lines[0].get_param('X'))
    for i in range(len(parsed_gcode.lines)):
        a = Movement()
        a.sz = sz
        a.sy = sy
        a.sx = sx
        if (parsed_gcode.lines[i].get_param('F') != None):
            a.speed = parsed_gcode.lines[i].get_param('F')
        else:
            a.speed = 150
        #print(parsed_gcode.lines[i].command)
        if (parsed_gcode.lines[i].command[1] == 1):
            a.type = 'LINEAR'
        if (parsed_gcode.lines[i].get_param('X') != None) and (parsed_gcode.lines[i].get_param('Y') != None) and (parsed_gcode.lines[i].get_param('Z') != None):
            fw.write((str(parsed_gcode.lines[i].get_param('X') * weight)) + ',' + (str(parsed_gcode.lines[i].get_param('Y') * weight)) + ',' + (str(parsed_gcode.lines[i].get_param('Z') * weight)) + ',' + str(weight) + '\n')
            a.ex = parsed_gcode.lines[i].get_param('X')
            a.ey = parsed_gcode.lines[i].get_param('Y')
            a.ez = parsed_gcode.lines[i].get_param('Z')
            sx = parsed_gcode.lines[i].get_param('X')
            sy = parsed_gcode.lines[i].get_param('Y')
            sz = parsed_gcode.lines[i].get_param('Z')
            weight *= 1
        MoveList.append(a)

