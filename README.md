# Oil Bath Immersion Cooker

<p>
<p align="center">
    <img src="https://github.com/Swicano/swicano.github.io/blob/master/images/Oil_bath/3Render_Right.jpg?raw=true" width="500" alt="Render of the completed proect"/> 
<p align="center"> The converted water bath with digital front panel </p>
</p>
 All the information regarding the construction and running of the "Oil Bath" built to convert an analog Thermo fisher water bath into a digital Sous Vide machine.  This project was superseded by the purchase of a commercial immersion circulator, and was repurposed to use higher temperature liquids due to the bath's metal construction and isolation of the heated area from the electronics (though it also means no circulator)


<p>
<p align="center">
    <img src="https://github.com/Swicano/swicano.github.io/blob/master/images/Oil_bath/OilbathOn-GalleryImage.jpg" width="425" alt="Flash Off-showing the reflected view off of the one-way mirror"/> <img src="https://github.com/Swicano/swicano.github.io/blob/master/images/Oil_bath/OilbathFlash-GalleryImage.jpg" width="425" alt="Flash On-showing the internal view past the one-way mirror"/> 
<p align="center"> The conversion in place on the old bath, with electronics hidden behind a one way mirror </p>
</p>


This project started in early 2016 when immersion circulators such as Anova's were still in the $200 range, and required the additional purchase of a bath to insert them into. From my experience using them for synthesizing gold nanorods, I knew lab water baths could do the job just as well, and used lab equipment can be had for a song. The only downside was that the models showing up on Ebay were analog only but I purchased a Precision Water Bath 182 (it had precision in the name, so it must be precise) for $35 shipped anyway. Upon arrival I was surprised at the elegant minimalism of it's guts. A heating resistor, an On/Off switch, and two adjustable thermal cutoffs wired directly to the mains was the entirety of it! 

Converting it from "set-it-and-forget-it" analog knobs to "SousVide status" was simple: turn all the cutoffs temperatures to max, add a solid state relay controlled extension cord and use an arduino to drive the relay, according to some feedback from a PTD. 

It existed in this breadboarded (eventually protoboarded) state for nearly 2 years, fully functional, and seeing frequent use, but not exactly beautiful.

<p>
<p align="center">
    <img src="https://github.com/Swicano/swicano.github.io/blob/master/images/Oil_bath/1_Breadboard1.jpg" height="425" alt="the brains of the operation"/> <img src="https://github.com/Swicano/swicano.github.io/blob/master/images/Oil_bath/2_Protoboard2.jpg" height="425" alt="The PTD circuit"/> 
<p align="center"> The "good enough" conversion (not pictured: the SSR driving the bath) </p>
</p>

But I was moving soon, and my new roommates would not appreciate the half-deconstructed nature of it, so I set out to integrate the new control circuitry into the bath. I decided to do the conversion in a reversible way, and with the electronics still visible, in homage to how it had existed for 2 years of it's second life. As well, I wanted to keep the big clicky power switch, I love big clicky switches that directly cut mains power, rather than those little soft buttons that are only doing it in software.

The cleaned up bath took a few months of intermittent 3D printing and test fitting, and last minute design changes (such as switching to the Max31865 to read the RTD) to come together, but by the time it did, it was a beauty to behold. Unfortunately, in the same span, a $70 Anova Nano appeared at my house. I dont have a need for two water immersion heaters, so this old bath has been relegated to experimental duty: used to cook in salt, or incubate various yeast and lactobacilli projects, or confit. 
