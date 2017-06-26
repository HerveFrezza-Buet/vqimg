# vqimg

This package is a ros package that offers nodes for image processing using vq2 (see my repository for this).

# gngt_node

It gets an image (/image_in), and applies a thresholding. The points above the threshold are sent to GNG-T, that build a graph.

/image_out publishes the image of that graph.

/component_centers publishes the center of each connected component. Thes centers are displayed on /image_out if some subscribers are reading /component_centers



