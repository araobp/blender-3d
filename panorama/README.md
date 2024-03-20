# Panorama

This folder contains pictures shot in this Blender scene ["Azuma House"](https://github.com/araobp/blender-3d/tree/main/scenes/AzumaHouse).

### Equirectangular

This is a simulation of a 360 degree 8K photo image in the Equirectangular projection.

```
Composition nodes:

[Render Layers] -> [Glare] -> [Composite]

<Glare parameters>
Fog Glow, High, Size: 6
```

https://araobp.github.io/blender-3d/panorama/

### Fisheye Equisolid 250 degrees

The other day, I visited a shooting location where a 240-degree panoramic video was being filmed using the combination of [Entaniya H250](https://products.entaniya.co.jp/en/list/hal-250-series/) and [Blackmagic URSA Mini Pro 12K (with OLPF)](https://www.blackmagicdesign.com/products/blackmagicursaminipro) cameras. Below is a simulation of the shooting using a combination of a Fisheye 250-degree lens and an 8K image sensor.

```
Composition nodes:

[Render Layers] -> [Image Distortion] -> [Glare] -> [Composite]

<Image Distortion paramters>
Dispersion: 0.02

<Glare parameters>
Fog Glow, High, Size: 6
```

<img src="fisheye_equisolid/AzumaHouse1.png" widh=800>

