"""Display material only; never changes pallet collision/geometry or perception."""
import re
import xml.etree.ElementTree as ET
PALLET_COLOR_HEX='#25A18E'


def rgba(hex_color=PALLET_COLOR_HEX):
    if not re.fullmatch(r'#[0-9a-fA-F]{6}',hex_color):raise ValueError('Expected #RRGGBB')
    return tuple(int(hex_color[i:i+2],16)/255. for i in (1,3,5))+(1.,)


def set_pallet_materials(root,hex_color=PALLET_COLOR_HEX):
    color=' '.join(format(v,'.12g') for v in rgba(hex_color));count=0
    for visual in root.findall('.//model[@name="single_floor_pallet"]/link/visual'):
        material=visual.find('material')
        if material is None:material=ET.SubElement(visual,'material')
        for name in ('ambient','diffuse'):
            entry=material.find(name)
            if entry is None:entry=ET.SubElement(material,name)
            entry.text=color
        count+=1
    if count==0:raise ValueError('No pallet visuals found')
    return count
