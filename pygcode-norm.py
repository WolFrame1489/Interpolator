#!C:\Python\python.exe

# Script to take (theoretically) any g-code file as input, and output a
# normalized version of it.
#
# Script outcome can have cursory verification with:
#   https://nraynaud.github.io/webgcode/

import argparse
import re
from collections import defaultdict
from contextlib import contextmanager
import os
f = open(os.getcwd() + '\\' + 'coderework.txt', 'w')
for pygcode_lib_type in ('installed_lib', 'relative_lib'):
    try:
        # pygcode
        from pygcode import Word
        from pygcode import Machine, Mode, Line
        from pygcode import GCodeArcMove, GCodeArcMoveCW, GCodeArcMoveCCW
        from pygcode import GCodeCannedCycle
        from pygcode import GCodeRapidMove, GCodeStopSpindle, GCodeAbsoluteDistanceMode
        from pygcode import split_gcodes
        from pygcode import Comment
        from pygcode.transform import linearize_arc, simplify_canned_cycle
        from pygcode.transform import ArcLinearizeInside, ArcLinearizeOutside, ArcLinearizeMid
        from pygcode.gcodes import _subclasses
        from pygcode import utils
        from pygcode.exceptions import MachineInvalidState

    except ImportError:
        import sys, os, inspect
        # Add pygcode (relative to this test-path) to the system path
        _this_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        sys.path.insert(0, os.path.join(_this_path, '..', 'src'))
        if pygcode_lib_type == 'installed_lib':
            continue # import was attempted before sys.path addition. retry import
        raise # otherwise the raised ImportError is a genuine problem
    break


# =================== Command Line Arguments ===================
# --- Types
def arc_lin_method_type(value):
    """
    :return: {Word('G2'): <linearize method class>, ... }
    """
    ARC_LIN_CLASS_MAP = {
        'i': ArcLinearizeInside,
        'o': ArcLinearizeOutside,
        'm': ArcLinearizeMid,
    }

    value_dict = defaultdict(lambda: ArcLinearizeMid)
    if value:
        match = re.search(r'^(?P<g2>[iom])(,(?P<g3>[iom]))?$', value, re.IGNORECASE)
        if not match:
            raise argparse.ArgumentTypeError("invalid format '%s'" % value)

        value_dict = {
            Word('g2'): ARC_LIN_CLASS_MAP[match.group('g2')],
            Word('g3'): ARC_LIN_CLASS_MAP[match.group('g2')],
        }
        if match.group('g3'):
            value_dict[Word('g3')] = ARC_LIN_CLASS_MAP[match.group('g3')]

    return value_dict

def word_list_type(value):
    """
    Convert csv string list into Word instances.
    >>> word_list_type("G73,G89") == set([Word('G73'), Word('G89')])
    :return: set of Word instances
    """
    canned_code_words = set()
    for word_str in re.split(r'\s*,\s*', value):
        canned_code_words.add(Word(word_str))

    return canned_code_words


# --- Defaults
DEFAULT_PRECISION = 0.005  # mm
DEFAULT_MACHINE_MODE = 'G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0'
DEFAULT_ARC_LIN_METHOD = 'm'
DEFAULT_CANNED_CODES = ','.join(str(w) for w in sorted(c.word_key for c in _subclasses(GCodeCannedCycle) if c.word_key))
# --- Create Parser
parser = argparse.ArgumentParser(
    description="Normalize gcode for machine consistency when using different CAM software."
)
parser.add_argument(
    'infile', type=argparse.FileType('r'),
    help="Gcode file to normalize.",
)

parser.add_argument(
    '--singles', '-s', dest='singles',
    action='store_const', const=True, default=False,
    help="Only output one command per gcode line.",
)
parser.add_argument(
    '--full', '-f', dest='full',
    action='store_const', const=True, default=False,
    help="Output full commands, any modal parameters will be acompanied with "
         "the fully qualified gcode command.",
)

# Machine
parser.add_argument(
    '--machine_mode', '-mm', dest='machine_mode', default=DEFAULT_MACHINE_MODE,
    help="Machine's startup mode as gcode (default: '%s')." % DEFAULT_MACHINE_MODE,
)

# Arc Linearizing
group = parser.add_argument_group(
    "Arc Linearizing",
    "Converting arcs (G2/G3 codes) into linear interpolations (G1 codes) to "
    "aproximate the original arc. Indistinguishable from an original arc when "
    "--arc_precision is set low enough."
)
group.add_argument(
    '--arc_linearize', '-al', dest='arc_linearize',
    action='store_const', const=True, default=False,
    help="Convert G2,G3 commands to a series of linear interpolations (G1 codes).",
)
group.add_argument(
    '--arc_lin_method', '-alm', dest='arc_lin_method',
    type=arc_lin_method_type, default=DEFAULT_ARC_LIN_METHOD,
    help="Method of linearizing arcs, i=inner, o=outer, m=mid. List 2 "
         "for <cw>,<ccw>, eg 'i,o'. 'i' is equivalent to 'i,i'. "
         "(default: '%s')." % DEFAULT_ARC_LIN_METHOD,
    metavar='{i,o,m}[,{i,o,m}]',
)
group.add_argument(
    '--arc_precision', '-alp', dest='arc_precision', type=float, default=DEFAULT_PRECISION,
    help="Maximum positional error when creating linear interpolation codes "
         "(default: %g)." % DEFAULT_PRECISION,
)

#parser.add_argument(
#    '--arc_alignment', '-aa', dest='arc_alignment', type=str, choices=('XYZ','IJK','R'),
#    default=None,
#    help="enforce precision on arcs, if XYZ the destination is altered to match the radius"
#         "if IJK or R then the arc'c centre point is moved to assure precision",
#)

# Canned Cycles
group = parser.add_argument_group(
    "Canned Cycle Simplification",
    "Convert canned cycles into basic linear or scalar codes, such as linear "
    "interpolation (G1), and pauses (or 'dwells', G4)"
)
group.add_argument(
    '--canned_expand', '-ce', dest='canned_expand',
    action='store_const', const=True, default=False,
    help="Expand canned cycles into basic linear movements, and pauses.",
)
group.add_argument(
    '--canned_codes', '-cc', dest='canned_codes',
    type=word_list_type, default=DEFAULT_CANNED_CODES,
    help="List of canned gcodes to expand, (default is '%s')." % DEFAULT_CANNED_CODES,
)

# Finalize Code
group = parser.add_argument_group(
    "Final Machine Actions",
    "standardize what's done at the end of a gcode program."
)
group.add_argument(
    '--zero_xy', '-zxy', dest="zero_xy",
    action='store_const', const=True, default=False,
    help="On completion, move straight up to rapid_safety_height, "
         "then across to X0 Y0.",
)
group.add_argument(
    '--zero_z', '-zz', dest="zero_z",
    action='store_const', const=True, default=False,
    help="On completion, move down to Z0 (done after zero_xy, if set).",
)
group.add_argument(
    '--rapid_safety_height', '-rsh', dest="rapid_safety_height",
    type=float, default=None,
    help="Z value to move to before traversing workpiece (if not set, max "
         "value will be attempted).",
)
group.add_argument(
    '--spindle_off', '-so', dest="spindle_off",
    action='store_const', const=True, default=False,
    help="On completion, turn spindle off.",
)

# Removing non-functional content
group = parser.add_argument_group(
    "Removing Content",
    "options for the removal of content."
)
group.add_argument(
    '--rm_comments', '-rc', dest='rm_comments',
    action='store_const', const=True, default=False,
    help="Remove all comments (non-functional).",
)
group.add_argument(
    '--rm_blanks', '-rb', dest='rm_blanks',
    action='store_const', const=True, default=False,
    help="Remove all empty lines (non-functional).",
)
group.add_argument(
    '--rm_whitespace', '-rws', dest='rm_whitespace',
    action='store_const', const=True, default=False,
    help="Remove all whitespace from gcode blocks (non-functional).",
)
group.add_argument(
    '--rm_gcodes', '-rmg', dest='rm_gcodes',
    type=word_list_type, default=[],
    help="Remove gcode (and it's parameters) with words in the given list "
         "(eg: M6,G43) (note: only works for modal params with --full)",
)
group.add_argument(
    '--rm_invalid_modal', '-rmim', dest='rm_invalid_modal',
    action='store_const', const=True, default=False,
    help="Simply remove everything that isn't understood. Use with caution.",
)

# --- Parse Arguments
args = parser.parse_args()


# =================== Create Virtual CNC Machine ===================
class MyMode(Mode):
    default_mode = args.machine_mode

class MyMachine(Machine):
    MODE_CLASS = MyMode
    ignore_invalid_modal = args.rm_invalid_modal

machine = MyMachine()

# =================== Utility Functions ===================
omit_redundant_modes = utils.omit_redundant_modes
if args.full:
    omit_redundant_modes = lambda gcode_iter: gcode_iter  # bypass

def write(gcodes, modal_params=tuple(), comment=None, macro=None):
    global f
    """
    Write to output, while enforcing the flags:
        args.singles
        args.rm_comments
        args.rm_blanks
        args.rm_whitespace
    :param obj: Line, Block, GCode or Comment instance
    """
    assert not(args.full and modal_params), "with full specified, this should never be called with modal_params"
    if args.singles and len(gcodes) > 1:
        for g in sorted(gcodes):
            write([g], comment=comment)

    else:
        # remove comments
        if args.rm_comments:
            comment = None
        # remove particular gcodes
        if args.rm_gcodes:
            gcodes = [g for g in gcodes if g.word not in args.rm_gcodes]

        # Convert to string & write to file (or stdout)
        block_str = ' '.join(str(x) for x in (list(gcodes) + list(modal_params)))
        if args.rm_whitespace:
            block_str = re.sub(r'\s', '', block_str)
            f.write(block_str + '\n')
        line_list = []
        if block_str:
            line_list.append(block_str)
        if comment:
            line_list.append(str(comment))
        if macro:
            line_list.append(str(macro))
        line_str = ' '.join(line_list)
        f.write(line_str + '\n')
        if line_str or not args.rm_blanks:
            print('LINESTR', line_str)
            if 'G01' not in line_str:
                f.write('G01 ' + line_str + '\n')
            else:
                f.write(line_str + '\n')


def gcodes2str(gcodes):
    return ' '.join("%s" % g for g in gcodes)


@contextmanager
def split_and_process(gcode_list, gcode_class, comment):
    """
    Split gcodes by given class, yields given class instance
    :param gcode_list: list of GCode instances
    :param gcode_class: class inheriting from GCode (directly, or indirectly)
    :param comment: Comment instance, or None
    """
    (befores, (g,), afters) = split_gcodes(gcode_list, gcode_class)
    # write & process those before gcode_class instance
    if befores:
        write(befores)
        machine.process_gcodes(*befores)
    # yield, then process gcode_class instance
    yield g
    machine.process_gcodes(g)
    # write & process those after gcode_class instance
    if afters:
        write(afters)
        machine.process_gcodes(*afters)
    # write comment (if given)
    if comment:
        write([], comment=line.comment)



# =================== Process File ===================
for line_str in args.infile.readlines():
    line = Line(line_str)

    if args.rm_invalid_modal:
        machine.clean_block(line.block)

    # Effective G-Codes:
    #   fills in missing motion modal gcodes (using machine's current motion mode).
    effective_gcodes = machine.block_modal_gcodes(line.block)

    if args.arc_linearize and any(isinstance(g, GCodeArcMove) for g in effective_gcodes):
        with split_and_process(effective_gcodes, GCodeArcMove, line.comment) as arc:
            write([], comment=Comment("linearized arc: %r" % arc))
            linearize_params = {
                'arc_gcode': arc,
                'start_pos': machine.pos,
                'plane': machine.mode.plane_selection,
                'method_class': args.arc_lin_method[arc.word],
                'dist_mode': machine.mode.distance,
                'arc_dist_mode': machine.mode.arc_ijk_distance,
                'max_error': args.arc_precision,
                'decimal_places': 3,
            }
            for linear_gcode in omit_redundant_modes(linearize_arc(**linearize_params)):
                write([linear_gcode])

    elif args.canned_expand and any((g.word in args.canned_codes) for g in effective_gcodes):
        with split_and_process(effective_gcodes, GCodeCannedCycle, line.comment) as canned:
            write([], comment=Comment("expanded: %r" % canned))
            simplify_canned_params = {
                'canned_gcode': canned,
                'start_pos': machine.pos,
                'plane': machine.mode.plane_selection,
                'dist_mode': machine.mode.distance,
                'axes': machine.axes,
            }
            for simplified_gcode in omit_redundant_modes(simplify_canned_cycle(**simplify_canned_params)):
                write([simplified_gcode])

    else:
        if args.full:
            write(effective_gcodes, comment=line.comment, macro=line.macro)
        else:
            write(line.block.gcodes, modal_params=line.block.modal_params, comment=line.comment, macro=line.macro)
        machine.process_block(line.block)

# Finalizing Motion & Spindle
if any([args.spindle_off, args.zero_xy, args.zero_z]):
    write([], comment=Comment("pygcode-norm.py: finalizing"))
if any([args.zero_xy, args.zero_z]) and not(isinstance(machine.mode.distance, GCodeAbsoluteDistanceMode)):
    write([GCodeAbsoluteDistanceMode()])
if args.spindle_off:
    write([GCodeStopSpindle()], comment=Comment("spindle off"))

if args.zero_xy:
    rapid_safety_height = args.rapid_safety_height
    if rapid_safety_height is None:
        rapid_safety_height = machine.abs2work(machine.abs_range_max).Z

if args.zero_xy:
    write([GCodeRapidMove(Z=rapid_safety_height)], comment=Comment("move to safe height"))
    write([GCodeRapidMove(X=0, Y=0)], comment=Comment("move to planar origin"))

if args.zero_z:
    write([GCodeRapidMove(Z=0)], comment=Comment("move to zero height"))
