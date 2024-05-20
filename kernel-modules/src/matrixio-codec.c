/*
 * matrix-codec.c -- MATRIX microphone array audio driver
 *
 * Copyright 2017-2020 MATRIX Labs
 *
 * Author: Andres Calderon <andres.calderon@admobilize.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "matrixio-core.h"
#include "matrixio-pcm.h"

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5,3,0)
static struct snd_soc_dai_link matrixio_snd_soc_dai[] = {
    {
	.name = "matrixio.mic.0",
	.stream_name = "matrixio.mic.0",
	.codec_dai_name = "snd-soc-dummy-dai",
	.cpu_dai_name = "matrixio-mic.0",
	.platform_name = "matrixio-mic",
	.codec_name = "snd-soc-dummy",
    },
    {
	.name = "matrixio.pcm-out.0",
	.stream_name = "matrixio.pcm-out.0",
	.codec_dai_name = "snd-soc-dummy-dai",
	.cpu_dai_name = "matrixio-pcm-out.0",
	.platform_name = "matrixio-playback",
	.codec_name = "snd-soc-dummy",
    }};
#else
SND_SOC_DAILINK_DEFS(matrixio_mic,
	DAILINK_COMP_ARRAY(COMP_CPU("matrixio-mic.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("snd-soc-dummy", "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("matrixio-mic")));

SND_SOC_DAILINK_DEFS(matrixio_playback,
	DAILINK_COMP_ARRAY(COMP_CPU("matrixio-pcm-out.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("snd-soc-dummy", "snd-soc-dummy-dai")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("matrixio-playback")));

static struct snd_soc_dai_link matrixio_snd_soc_dai[] = {
    {
	.name = "matrixio.mic.0",
	.stream_name = "matrixio.mic.0",
	SND_SOC_DAILINK_REG(matrixio_mic),
    },
    {
	.name = "matrixio.pcm-out.0",
	.stream_name = "matrixio.pcm-out.0",
	SND_SOC_DAILINK_REG(matrixio_playback),
    }};
#endif

static struct snd_soc_card matrixio_soc_card = {
    .name = "MATRIXIO-SOUND",
    .owner = THIS_MODULE,
    .dai_link = matrixio_snd_soc_dai,
    .num_links = ARRAY_SIZE(matrixio_snd_soc_dai),
    .fully_routed = true,
};

static const struct snd_kcontrol_new matrixio_snd_controls[] = {};

static const struct snd_soc_dapm_widget matrixio_dapm_widgets[] = {};

static const struct snd_soc_dapm_route matrixio_dapm_routes[] = {};

static int matrixio_codec_probe(struct snd_soc_component *codec) { return 0; }

static const struct snd_soc_component_driver matrixio_soc_codec_driver = {

    .probe = matrixio_codec_probe,
    .controls = matrixio_snd_controls,
    .num_controls = ARRAY_SIZE(matrixio_snd_controls),
    .dapm_widgets = matrixio_dapm_widgets,
    .num_dapm_widgets = ARRAY_SIZE(matrixio_dapm_widgets),
    .dapm_routes = matrixio_dapm_routes,
    .num_dapm_routes = ARRAY_SIZE(matrixio_dapm_routes),
};

static struct snd_soc_dai_driver matrixio_dai_driver[] = {
    {
	.name = "matrixio-pcm-out.0",
	.playback =
	    {
		.stream_name = "matrixio-pcm-out.0",
		.channels_min = 2,
		.channels_max = 2,
		.rates = MATRIXIO_RATES,
		.rate_min = 8000,
		.rate_max = 48000,
		.formats = MATRIXIO_FORMATS,
	    },
    },
    {
	.name = "matrixio-mic.0",
	.capture =
	    {
		.stream_name = "matrixio-mic.0",
		.channels_min = 1,
		.channels_max = MATRIXIO_CHANNELS_MAX,
		.rates = MATRIXIO_RATES,
		.rate_min = 8000,
		.rate_max = 96000,
		.formats = MATRIXIO_FORMATS,
	    },
    }};

static int matrixio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &matrixio_soc_card;
	int ret;

	card->dev = &pdev->dev;

	ret = devm_snd_soc_register_component(
	    &pdev->dev, &matrixio_soc_codec_driver, matrixio_dai_driver,
	    ARRAY_SIZE(matrixio_dai_driver));
	if (ret) {
		dev_err(&pdev->dev, "Failed to register MATRIXIO codec: %d\n",
			ret);
		return ret;
	}

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register MATRIXIO card (%d)\n",
			ret);
		return ret;
	}

	return ret;
}

static int matrixio_codec_remove(struct platform_device *pdev) { return 0; }

static const struct of_device_id snd_matrixio_codec_of_match[] = {
    {
	.compatible = "matrixio-codec",
    },
    {},
};
MODULE_DEVICE_TABLE(of, snd_matrixio_codec_of_match);

static struct platform_driver matrixio_codec_driver = {
    .driver =
	{
	    .name = "matrixio-codec",
	    .owner = THIS_MODULE,
	    .of_match_table = of_match_ptr(snd_matrixio_codec_of_match),
	    .pm = &snd_soc_pm_ops,
	},
    .probe = matrixio_probe,
    .remove = matrixio_codec_remove,
};

module_platform_driver(matrixio_codec_driver);

MODULE_SOFTDEP("pre: matrixio-mic");
MODULE_SOFTDEP("pre: matrixio-playback");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andres Calderon <andres.calderon@admobilize.com>");
MODULE_DESCRIPTION("MATRIXIO audio module");
