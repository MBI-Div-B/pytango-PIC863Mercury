from .PIC863Mercury import PIC863Mercury


def main():
    import sys
    import tango.server

    args = ["PIC863Mercury"] + sys.argv[1:]
    tango.server.run((PIC863Mercury,), args=args)
