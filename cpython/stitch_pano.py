'''
from StereoPi.sh :
    pto_gen --projection=2 --fov=360 -o ./tmp/project.pto $1 $2
    pto_template --output=./tmp/project.pto --template=template.pto ./tmp/project.pto
    hugin_executor --stitching --prefix=$RESULT_NAME ./tmp/project.pto

additional parameters:
    pto_lensstack -o project1.pto --new-lens i1 project.pto
    cpfind -o project.pto --multirow project.pto
    cpclean -o project.pto project.pto
    linefind -o project.pto project.pto
    pto_var -o project.pto --opt TrX,TrY,TrZ,r,Eev,Ra,Rb,Rc,Rd,Re,!TrX0,!TrY0,!TrZ0,!r0,!Eev0,!Ra1,!Rb1,!Rc1,!Rd1,!Re1 project.pto
    autooptimiser -n -o project.pto project.pto
    pano_modify  --projection=1 --fov=AUTO --center --canvas=AUTO --crop=AUTO -o project.pto project.pto
'''

import os
import subprocess
import shutil
import glob


def stitch_pano(image_dir):
    base_dir = os.path.dirname(image_dir)
    print("basedir:", base_dir)
    tmp_dir = os.path.join(base_dir, "tmp")
    output = os.path.join(base_dir, "pano")

    # Get all jpg files in the directory
    files = sorted(glob.glob(os.path.join(image_dir, '*.jpg')))
    print(len(files), "images found.")

    # Create a temporary directory
    os.makedirs(tmp_dir, exist_ok=True)
    # generate a project from template and execute stitching

    # create Hugin project, match to template, and stitch
    calls = [['pto_gen', '--projection=2', '--fov=360', '-o', f'./{tmp_dir}/project.pto', files[0], files[1]],  
             ['pto_template', f'--output=./{tmp_dir}/project.pto', f'--template={base_dir}/template.pto', f'./{tmp_dir}/project.pto'],
             ['hugin_executor', '--stitching', f'--prefix={output}', f'./{tmp_dir}/project.pto']]
    for call in calls:
        subprocess.run(call)

    # remove temporary directory
    shutil.rmtree(tmp_dir)


if __name__ == "__main__":
    stitch_pano("panocam/images")
