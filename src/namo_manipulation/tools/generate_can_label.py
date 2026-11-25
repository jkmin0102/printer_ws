#!/usr/bin/env python3
"""Create an explicitly approximate cola label and a visual-only UV side mesh.

This is code-generated artwork, not a photographed or official product label.
It changes rendered appearance only; YOLO still receives the actual camera RGB.
Cylinder collision geometry, mass, dimensions and detector outputs are untouched.
"""

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT/'models'
SIZE = 1024
FONT = Path('/usr/share/fonts/type1/gsfonts/z003034l.pfb')


def artwork():
    MODELS.mkdir(parents=True, exist_ok=True)
    svg = '''<svg xmlns="http://www.w3.org/2000/svg" width="1024" height="1024" viewBox="0 0 1024 1024">
<metadata>Generated visual approximation for Gazebo. Not an official or photographed Coca-Cola label.</metadata>
<rect width="1024" height="1024" fill="#d40016"/>
<g transform="translate(512,512) rotate(-90)">
<text x="0" y="65" text-anchor="middle" fill="white" font-family="URW Chancery L" font-size="244">Coca-Cola</text>
</g>
<path d="M 705 0 C 640 190 815 350 720 520 C 625 690 645 850 760 1024" fill="none" stroke="white" stroke-width="23"/>
<path d="M 746 0 C 681 190 856 350 761 520 C 666 690 686 850 801 1024" fill="none" stroke="white" stroke-width="7"/>
<text x="512" y="48" text-anchor="middle" fill="white" font-family="Liberation Sans" font-size="25">ORIGINAL TASTE</text>
</svg>'''
    (MODELS/'cola_label.svg').write_text(svg)
    image = Image.new('RGB', (SIZE, SIZE), (212, 0, 22))
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype(str(FONT), 244)
    bounds = font.getbbox('Coca-Cola')
    word = Image.new('RGBA', (bounds[2]-bounds[0]+20, bounds[3]-bounds[1]+20), (0, 0, 0, 0))
    ImageDraw.Draw(word).text((10-bounds[0], 10-bounds[1]), 'Coca-Cola', font=font, fill='white')
    word = word.rotate(90, expand=True, resample=Image.Resampling.BICUBIC)
    image.paste(word, ((SIZE-word.width)//2, (SIZE-word.height)//2), word)

    def curve(points, shift=0.):
        result = []
        for i in range(201):
            t = i/200
            x = sum(math.comb(3, j)*(1-t)**(3-j)*t**j*points[j][0] for j in range(4))
            y = sum(math.comb(3, j)*(1-t)**(3-j)*t**j*points[j][1] for j in range(4))
            result.append((x+shift, y))
        return result
    ribbon = curve([(705, 0), (640, 190), (815, 350), (720, 520)])
    ribbon += curve([(720, 520), (625, 690), (645, 850), (760, 1024)])
    draw.polygon([(x-11.5, y) for x, y in ribbon]
                 +[(x+11.5, y) for x, y in reversed(ribbon)], fill='white')
    draw.polygon([(x+37.5, y) for x, y in ribbon]
                 +[(x+44.5, y) for x, y in reversed(ribbon)], fill='white')
    small = ImageFont.truetype('/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf', 25)
    draw.text((512, 35), 'ORIGINAL TASTE', font=small, anchor='mm', fill='white')
    image.save(MODELS/'cola_label.png')


def side_mesh():
    positions, normals, uvs, indices = [], [], [], []
    count, radius, half_height = 96, .025, .0655
    for index in range(count):
        a, b = 2*math.pi*index/count, 2*math.pi*(index+1)/count
        corners = [(a, -half_height, index/count, 0.),
                   (b, -half_height, (index+1)/count, 0.),
                   (b, half_height, (index+1)/count, 1.),
                   (a, half_height, index/count, 1.)]
        for angle, z, u, v in corners:
            positions.extend((radius*math.cos(angle), radius*math.sin(angle), z))
            normals.extend((math.cos(angle), math.sin(angle), 0.))
            uvs.extend((u, v))
        for corner in (0, 1, 2, 0, 2, 3):
            i = 4*index+corner
            indices.extend((i, i, i))
    def source(name, values, stride, components):
        return f'''<source id="{name}"><float_array id="{name}-array" count="{len(values)}">{' '.join(f'{v:.9g}' for v in values)}</float_array>
<technique_common><accessor source="#{name}-array" count="{len(values)//stride}" stride="{stride}">{''.join(f'<param name="{c}" type="float"/>' for c in components)}</accessor></technique_common></source>'''
    dae = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><contributor><authoring_tool>NAMO code-generated visual approximation</authoring_tool></contributor><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
<library_images><image id="cola-image"><init_from>cola_label.png</init_from></image></library_images>
<library_effects><effect id="cola-effect"><profile_COMMON>
<newparam sid="cola-surface"><surface type="2D"><init_from>cola-image</init_from></surface></newparam>
<newparam sid="cola-sampler"><sampler2D><source>cola-surface</source><wrap_s>WRAP</wrap_s><wrap_t>CLAMP</wrap_t></sampler2D></newparam>
<technique sid="common"><phong><emission><color>0 0 0 1</color></emission><ambient><color>1 1 1 1</color></ambient><diffuse><texture texture="cola-sampler" texcoord="UVSET0"/></diffuse><specular><color>.15 .15 .15 1</color></specular><shininess><float>12</float></shininess></phong></technique>
</profile_COMMON></effect></library_effects>
<library_materials><material id="cola-material" name="GeneratedColaApproximation"><instance_effect url="#cola-effect"/></material></library_materials>
<library_geometries><geometry id="cola-side"><mesh>
{source('positions', positions, 3, 'XYZ')}{source('normals', normals, 3, 'XYZ')}{source('uvs', uvs, 2, 'ST')}
<vertices id="cola-vertices"><input semantic="POSITION" source="#positions"/></vertices>
<triangles material="ColaMaterial" count="{count*2}"><input semantic="VERTEX" source="#cola-vertices" offset="0"/><input semantic="NORMAL" source="#normals" offset="1"/><input semantic="TEXCOORD" source="#uvs" offset="2" set="0"/><p>{' '.join(map(str, indices))}</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="Scene"><node id="CanLabel"><instance_geometry url="#cola-side"><bind_material><technique_common><instance_material symbol="ColaMaterial" target="#cola-material"><bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#Scene"/></scene></COLLADA>'''
    ET.fromstring(dae)
    (MODELS/'cola_side.dae').write_text(dae)
    (MODELS/'cola_label_metadata.json').write_text(json.dumps({
        'provenance': 'Code-generated visual approximation; not an official/photographed product label',
        'purpose': 'Visual-domain experiment; inference remains original best.pt on actual Gazebo RGB',
        'side_radius_m': radius, 'side_height_m': 2*half_height, 'triangles': count*2,
        'physical_collision_modified': False, 'font_source': str(FONT),
        'visible_front': 'negative X, label center u=0.5',
    }, indent=2))


if __name__ == '__main__':
    artwork()
    side_mesh()
    print('Generated visual-only cola_label.svg/png, cola_side.dae and provenance metadata')
