function I = get_camera_image(h)
    f = getframe(h);
    I = f.cdata;
end