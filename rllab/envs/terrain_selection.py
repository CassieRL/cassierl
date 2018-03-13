"""
Use a random agent to move Cassie on pre-selected terrain
"""

import os
import random
import random_agent
import sys

from shutil import copyfile


TERRAIN_DIR = '../../model/terrains'
XML_SRC = '../../model/cassie2d_stiff.xml'
XML_COPY = '../../model/cassie2d_stiff_tmp.xml'


def main(argv):
    # get a list of png file names
    terrainlist = os.listdir(TERRAIN_DIR)

    terrain_png = argv[0]

    # get a random terrain from the list if specified
    if terrain_png == 'random':
        random.seed()
        terrain_png = terrainlist[random.randint(0, len(terrainlist) - 1)]
    else:
        assert terrain_png in terrainlist, 'Terrain not found!'

    # make a copy of the xml and operate on the source
    copyfile(XML_SRC, XML_COPY)

    # write a height field with specified atrributes to the source
    attributes = {
        'size': '\'10 10 1 0.001\'',
        'file': '\'../terrains/' + terrain_png + '\'',
        'pos': '\'0 0 0\'',
        'rgba': '\'0.47 0.33 0.28 1\'',
        'condim': '\'3\'',
        'conaffinity': '\'7\''
    }
    add_hfield_xml(attributes, XML_COPY, XML_SRC)

    # perform random agent on the source xml file with the added height field
    random_agent.main()


def add_hfield_xml(attributes, infile, outfile):
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
                fout.write('    <hfield name=\'terrain\' size={} file={}/>\n'.format(attributes['size'], attributes['file']))

            # if meet <worldbody> tag, add a <geom> right after it
            if line.strip().replace(' ', '') == '<worldbody>':
                fout.write('    <geom type=\'hfield\' hfield=\'terrain\' pos={} rgba={} condim={} conaffinity={}/>\n'.format(attributes['pos'], attributes['rgba'], attributes['condim'], attributes['conaffinity']))

            line = fin.readline()

        fin.close()
        fout.close()


if __name__ == '__main__':
    assert len(sys.argv) == 2, 'Usage: terrain_selection.py <<image.png> or random>'
    try:
        main(sys.argv[1:])
    finally:
        if os.path.isfile(XML_COPY):
            os.remove(XML_SRC)
            os.rename(XML_COPY, XML_SRC)
