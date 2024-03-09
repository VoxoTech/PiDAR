from lib.file_utils import list_files
from lib.pano_utils import hugin_stitch


if __name__ == "__main__":
    # Get all jpg files in the directory
    files = list_files("images", ext="jpg")
    print(len(files), "images found.")

    files = [files[0], files[2], files[4], files[6]]

    hugin_stitch(files, 
                 template="panocam/template_4.pto",
                 width=3600,
                 output_path="export/pano.jpg",
                 tmp_dir="panocam/tmp")
