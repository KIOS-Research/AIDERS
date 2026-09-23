import torch.nn as nn
import torch.nn.functional as F


class convBlock(nn.Module):
    def __init__(
        self,
        in_channels: int,
        out_channels: int,
        kernel_size: int = 3,
        stride: int = 1,
        padding: int = 1,
        norm: bool = False,
    ) -> None:
        super(convBlock, self).__init__()
        self.norm = norm
        self.conv = nn.Conv2d(
            in_channels=in_channels,
            out_channels=out_channels,
            kernel_size=kernel_size,
            stride=stride,
            padding=padding,
        )
        if norm:
            self.norm = nn.InstanceNorm2d(in_channels, momentum=0.01, affine=True)
        self.activation = nn.LeakyReLU(0.1)

    def forward(self, x):
        x = self.conv(x)
        if self.norm:
            x = self.norm(x)
        x = self.activation(x)

        return x


class ESPCN(nn.Module):
    def __init__(self, input_channels, upscale_factor):
        super(ESPCN, self).__init__()
        self.conv1 = convBlock(
            in_channels=input_channels,
            out_channels=input_channels,
            kernel_size=1,
            stride=1,
            padding=0,
            norm=True,
        )

        self.conv2 = convBlock(
            in_channels=input_channels,
            out_channels=128,
            kernel_size=5,
            stride=1,
            padding=2,
        )

        self.conv3 = convBlock(
            in_channels=128, out_channels=64, kernel_size=3, stride=1, padding=1
        )

        self.conv4 = convBlock(
            in_channels=64, out_channels=32, kernel_size=3, stride=1, padding=1
        )

        self.conv5 = convBlock(
            in_channels=32, out_channels=16, kernel_size=3, stride=1, padding=1
        )

        self.pixel_shuffle = nn.PixelShuffle(upscale_factor)

    def forward(self, x):
        x = self.conv1(x)

        x = self.conv2(x)

        x = self.conv3(x)

        x = self.conv4(x)

        x = self.conv5(x)

        x = self.pixel_shuffle(x)

        return x
