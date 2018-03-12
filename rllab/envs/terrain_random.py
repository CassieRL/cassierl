"""
Randomly selects a number of terrain from the terrain folder and add to XML.
"""

import os
import random
import sys

from shutil import copyfile


ALL_TERRAINS = os.listdir('../../model/terrains')
XML_SRC = '../../model/cassie2d_stiff.xml'
XML_COPY = '../../model/cassie2d_stiff_tmp.xml'

def main(argv):
    # make sure the user passes in an integer as an argument
    assert represent_int(argv[0]), 'NUM_TERRAINS must be an integer.'

    NUM_TERRAINS = int(argv[0])

    # make sure the number of terrains to random is positive.
    assert NUM_TERRAINS > 0, 'NUM_TERRAINS must be positive.'
    # make sure the terrain directory has enough terrains
    assert len(ALL_TERRAINS) >= NUM_TERRAINS, 'Not enough terrains.'

    terrainlist = []

    # randomly pick NUM_TERRAINS terrains out of the directory
    for _ in range(NUM_TERRAINS):
        random.seed()
        terrainlist.append(random.choice(ALL_TERRAINS))

    # make a copy of the xml and operate on the source
    copyfile(XML_SRC, XML_COPY)

    # write a height field with specified atrributes to the source
    attributes = {
        'size': '\'10 10 1 0.001\'',
        'file': '',
        'pos': '\'0 0 0\'',
        'rgba': '\'0.47 0.33 0.28 1\'',
        'condim': '\'3\'',
        'conaffinity': '\'7\''
    }

    # add randomly chosen terrains into xml one by one
    add_hfield_xml(attributes, terrainlist, XML_COPY, XML_SRC)


def add_hfield_xml(attributes, terrainlist, infile, outfile):
    """
    Writes a new  <hfield>  to  <asset>
           a new  <geom>  of type  <hfield>  to  <worldbody>
    """
    with open(infile, 'r') as fin, open(outfile, 'w') as fout:
        line = fin.readline()

        while line:
            fout.write(line)

            # if meet <asset> tag, add an <hfield> right after it
            if line.strip().replace(' ', '') == '<asset>':
                for i in range(len(terrainlist)):
                    attributes['file'] = '\'../terrains/' + terrainlist[i] + '\''
                    fout.write('    <hfield name=\'terrain{}\' size={} file={}/>\n'.format(i, attributes['size'], attributes['file']))

            # if meet <worldbody> tag, add a <geom> right after it
            if line.strip().replace(' ', '') == '<worldbody>':
                for i in range(len(terrainlist)):
                    fout.write('    <geom type=\'hfield\' hfield=\'terrain{}\' pos={} rgba={} condim={} conaffinity={}/>\n'.format(i, attributes['pos'], attributes['rgba'], attributes['condim'], attributes['conaffinity']))

            line = fin.readline()

        fin.close()
        fout.close()


def represent_int(s):
    """
    Returns true if string s represents an integer, false otherwise.
    """
    try:
        int(s)
        return True
    except ValueError:
        return False


if __name__ == '__main__':
    assert len(sys.argv) == 2, 'Usage: random_terrain.py NUM_TERRAINS'
    try:
        main(sys.argv[1:])
    finally:
        if os.path.isfile(XML_COPY):
            input('Press Enter to restore cassie2d_stiff.xml back to normal.')
            os.remove(XML_SRC)
            os.rename(XML_COPY, XML_SRC)
