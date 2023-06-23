---
title: Mechanical Design
layout: default
nav_order: 5
---

# Mechanical Design

## Designing Links
When designing the links, we should think of the different load types acting on the links. While the crossection of a link does not play any role if the link are under tension or compression, it is extremely important under bending. Bending loads are the most common load in a robot arm, and we should therefore design the links to be as stiff as possible under bending. The kinematic model of an robot arm typically does not account for bending of the links, thus any bending of the links will result in a deviation between the desired and actual end-effector position.
However, we should also consider the weight of the links. The weight of the links will affect the maximum payload of the robot, and the power consumption of the motors. The weight of the links will also affect the inertia of the links, which will affect the dynamic performance of the robot.
Therefore, we should not try to make the links stiff by adding more and more material, making them thick and heavy.
Instead, we should try to make the links stiff by designing them with a crossection that is resistant against bending; adding some material where it is needed, and removing material where it is not needed. We try to balance the stiffness and weight of the links.

The key concept behind this is the [**second moment of area**](https://en.wikipedia.org/wiki/Second_moment_of_area), also known as the area moment of inertia. The second moment of area is a measure of the cross-sectional shape of a beam. It is used in bending calculations, and also in determining the stiffness of a beam under bending. The second moment of area is defined as the integral of the area of a cross-section about an axis perpendicular to the plane of the cross-section. The second moment of area is a geometrical property of the cross-section, and is therefore independent of the material of the beam. The second moment of area is denoted $I$ and has the unit $m^4$.

<iframe width="560" height="315" src="https://www.youtube.com/embed/Bls5KnQOWkY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>


### Structural optimization in nature
{: .d-inline-block }
Optional
{: .label .label-blue } 

Efficiency is an evolutionary advantage.
Plants competing for sunlight try to grow tall quickly, but they also need to be stiff to withstand wind and rain. It is and balancing act: using material to grow tall or using material to become stiff.
Thus, a efficient plant that is as stiff as its competitors while using less material will have an advantage over its competitors. Plant stems are often hollow or U-shaped, leaves often bent or U-shaped to concentrate material far from the neutral axis to maximize the second moment of area.

<img src="https://upload.wikimedia.org/wikipedia/commons/7/7c/Brassica_napus_03_ies.jpg" width="325"> <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/9/9f/Os_longum_1.png/1599px-Os_longum_1.png?20200828160255" width="325">

Similarly, limbs of animals need to be stiff to be functional, but they should also be light to minimize energy usage.
Evolution has optimized the structure of bones to be as stiff as possible while using as little material as possible. Bones are hollow; material close to the neutral axis without significant contribution to stiffness under bending is removed.


## Designing Joints