
# import module
#from pdf2image import convert_from_path
import fitz
import sys
import time
import os

# start = time.time() 
 
# # Store Pdf with convert_from_path function
# images = convert_from_path(sys.argv[1], dpi=1000)
 
# for i in range(len(images)):
#     # Save pages as images in the pdf
#     images[i].save('page'+ str(i) +'.png', 'PNG')

# end = time.time()
# print(end - start)



print(sys.argv)


start = time.time()
try:
    os.mkdir(sys.argv[1] + 'pngs/')
except:
    print("PNG folder exists.")
for f in os.listdir(sys.argv[1]):
    if f.endswith("pdf"):
        doc = fitz.open(sys.argv[1]+f)
        page = doc.load_page(0)
        pixmap = page.get_pixmap(dpi=100)
        pixmap.save(sys.argv[1]+'pngs/'+f[:-4]+'.png', 'PNG')


end = time.time()
print(end - start)
