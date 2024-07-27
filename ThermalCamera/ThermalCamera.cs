using System.Device.I2c;
using System.Text;

namespace ThermalCamera;

public sealed class ThermalCamera : IDisposable
{
    private readonly I2cDevice _device;
    private readonly ParamsMlx _mlx;
    public const int TotPixels = 768;
    public const int TotColumns = 32;
    public const int TotRows = 24;
    private const int TotAuxData = 64;
    private const int FrameDataError = -8;

    private const ushort StatusRegister = 0x8000,
        ControlRegister = 0x800D,
        ConfigurationRegister = 0x800F,
        RamStartRegister = 0x0400,
        RamEndRegister = 0x06FF,
        AuxDataStartAddress = 0x0700,
        EePromStartAddress = 0x2400;

    public ThermalCamera()
    {
        _device = I2cBus.Create(1).CreateDevice(0x33);
        _mlx = new ParamsMlx();

        // var eepromData = new ushort[832];
        // ReadWordsFromRegister(EePromStartAddress, eepromData);
        //
        // ExtractVddParameters(eepromData);
        // ExtractPtatParameters(eepromData);
        // ExtractGainParameters(eepromData);
        // ExtractTgcParameters(eepromData);
        // ExtractResolutionParameters(eepromData);
        // ExtractKsTaParameters(eepromData);
        // ExtractKsToParameters(eepromData);
        // ExtractAlphaParameters(eepromData);
        // ExtractOffsetParameters(eepromData);
        // ExtractKtaPixelParameters(eepromData);
        // ExtractKvPixelParameters(eepromData);
        // ExtractCpParameters(eepromData);
        // ExtractCilcParameters(eepromData);
        // ExtractDeviatingPixels(eepromData);

        // Console.WriteLine("successfully extracted params from eeprom");
        // Console.WriteLine(_mlx.ToString());
    }

    private void ExtractVddParameters(ushort[] eepromData)
    {
        byte kVdd = (byte)((eepromData[51] & 0xFF00) >> 8);

        short vdd25 = (short)(eepromData[51] & 0x00FF);
        vdd25 = (short)(((vdd25 - 256) << 5) - 8192);

        _mlx.KVdd = (short)(32 * kVdd);
        _mlx.Vdd25 = vdd25;
    }

    private void ExtractPtatParameters(ushort[] eeData)
    {
        float kvPtat = (eeData[50] & 0xFC00) >> 10;
        if (kvPtat > 31)
        {
            kvPtat -= 64;
        }

        kvPtat /= 4096;

        float ktPtat = eeData[50] & 0x03FF;
        if (ktPtat > 511)
        {
            ktPtat -= 1024;
        }

        ktPtat /= 8;

        var vPtat25 = eeData[49];

        var alphaPtat = (float)((eeData[16] & 0xF000) / Math.Pow(2, 14) + 8.0f);

        _mlx.KvPtat = kvPtat;
        _mlx.KtPtat = ktPtat;
        _mlx.VPtat25 = vPtat25;
        _mlx.AlphaPtat = alphaPtat;
    }

    private void ExtractGainParameters(ushort[] eeData)
    {
        _mlx.GainEe = (short)eeData[48];
    }

    private void ExtractTgcParameters(ushort[] eeData)
    {
        _mlx.Tgc = (eeData[60] & 0x00FF) / 32.0f;
    }

    private void ExtractResolutionParameters(ushort[] eeData)
    {
        _mlx.ResolutionEe = (byte)((eeData[56] & 0x3000) >> 12);
    }

    private void ExtractKsTaParameters(ushort[] eeData)
    {
        _mlx.KsTa = (eeData[60] & 0x00FF) / 8192.0f;
    }

    private void ExtractKsToParameters(ushort[] eeData)
    {
        var step = (byte)(((eeData[63] & 0x3000) >> 12) * 10);

        _mlx.Ct[0] = -40;
        _mlx.Ct[1] = 0;
        // WTF ?
        // _mlx.Ct[2] = (short)((eeData[63] & 0x00F0) >> 4);
        // _mlx.Ct[3] = (short)((eeData[63] & 0x00F0) >> 4);
        _mlx.Ct[2] = (short)(_mlx.Ct[2] * step);
        _mlx.Ct[3] = (short)(_mlx.Ct[2] + _mlx.Ct[3] * step);
        _mlx.Ct[4] = 400;

        var ksToScale = (eeData[63] & 0x000F) + 8;
        ksToScale = (int)(1UL << ksToScale);

        _mlx.KsTo[0] = (eeData[61] & 0x00FF) / (float)ksToScale;
        _mlx.KsTo[1] = ((eeData[61] & 0xFF00) >> 8) / (float)ksToScale;
        _mlx.KsTo[2] = (eeData[62] & 0x00FF) / (float)ksToScale;
        _mlx.KsTo[3] = ((eeData[62] & 0xFF00) >> 8) / (float)ksToScale;
        _mlx.KsTo[4] = (float)-0.0002;
    }

    private void ExtractAlphaParameters(ushort[] eeData)
    {
        int[] accRow = new int[24];
        int[] accColumn = new int[32];
        int p;
        float[] alphaTemp = new float[TotPixels];


        var accRemScale = (byte)(eeData[32] & 0x000F);
        var accColumnScale = (byte)((eeData[32] & 0x00F0) >> 4);
        var accRowScale = (byte)((eeData[32] & 0x0F00) >> 8);
        int alphaRef = eeData[33];

        for (int i = 0; i < 6; i++)
        {
            p = i * 4;
            accRow[p + 0] = eeData[34 + i] & 0x000F;
            accRow[p + 1] = (eeData[34 + i] & 0x00F0) >> 4;
            accRow[p + 2] = (eeData[34 + i] & 0x0F00) >> 8;
            accRow[p + 3] = (eeData[34 + i] & 0xF000) >> 12;
        }

        for (int i = 0; i < TotRows; i++)
        {
            if (accRow[i] > 7)
            {
                accRow[i] -= 16;
            }
        }

        for (int i = 0; i < 8; i++)
        {
            p = i * 4;
            accColumn[p + 0] = eeData[40 + i] & 0x000F;
            accColumn[p + 1] = (eeData[40 + i] & 0x00F0) >> 4;
            accColumn[p + 2] = (eeData[40 + i] & 0x0F00) >> 8;
            accColumn[p + 3] = (eeData[40 + i] & 0xF000) >> 12;
        }

        for (int i = 0; i < TotColumns; i++)
        {
            if (accColumn[i] > 7)
            {
                accColumn[i] -= 16;
            }
        }

        for (int i = 0; i < TotRows; i++)
        {
            for (int j = 0; j < TotColumns; j++)
            {
                p = 32 * i + j;
                alphaTemp[p] = (eeData[64 + p] & 0x03F0) >> 4;
                if (alphaTemp[p] > 31)
                {
                    alphaTemp[p] -= 64;
                }

                alphaTemp[p] *= 1 << accRemScale;
                alphaTemp[p] = alphaRef + (accRow[i] << accRowScale) + (accColumn[j] << accColumnScale) +
                               alphaTemp[p];
                alphaTemp[p] /= (float)Math.Pow(2, 4);
                alphaTemp[p] -= _mlx.Tgc * (_mlx.CpAlpha[0] + _mlx.CpAlpha[1]) / 2;
                alphaTemp[p] = (float)(0.000001 / alphaTemp[p]);
            }
        }

        var temp = alphaTemp[0];
        for (int i = 1; i < TotPixels; i++)
        {
            if (alphaTemp[i] > temp)
            {
                temp = alphaTemp[i];
            }
        }

        byte alphaScale = 0;
        while (temp < 32767.4)
        {
            temp *= 2;
            alphaScale = (byte)(alphaScale + 1);
        }

        for (int i = 0; i < TotPixels; i++)
        {
            temp = (float)(alphaTemp[i] * Math.Pow(2, alphaScale));
            _mlx.Alpha[i] = (ushort)(temp + 0.5);
        }

        _mlx.AlphaScale = alphaScale;
    }

    private void ExtractOffsetParameters(ushort[] eeData)
    {
        int[] occRow = new int[24];
        int[] occColumn = new int[32];
        int p;

        byte occRemScale = (byte)(eeData[16] & 0x000F);
        byte occColumnScale = (byte)((eeData[16] & 0x00F0) >> 4);
        byte occRowScale = (byte)((eeData[16] & 0x0F00) >> 8);
        short offsetRef = (short)eeData[17];

        for (int i = 0; i < 6; i++)
        {
            p = i * 4;
            occRow[p + 0] = eeData[18] & 0x000F;
            occRow[p + 1] = (eeData[18] & 0x00F0) >> 4;
            occRow[p + 2] = (eeData[18 + i] & 0x0F00) >> 8;
            occRow[p + 3] = (eeData[18 + i] & 0xF000) >> 12;
        }

        for (int i = 0; i < TotRows; i++)
        {
            if (occRow[i] > 7)
            {
                occRow[i] -= 16;
            }
        }

        for (int i = 0; i < 8; i++)
        {
            p = i * 4;
            occColumn[p + 0] = eeData[24 + i] & 0x000F;
            occColumn[p + 1] = (eeData[24 + i] & 0x00F0) >> 4;
            occColumn[p + 2] = (eeData[24 + i] & 0x0F00) >> 8;
            occColumn[p + 3] = (eeData[24 + i] & 0xF000) >> 12;
        }

        for (int i = 0; i < TotColumns; i++)
        {
            if (occColumn[i] > 7)
            {
                occColumn[i] -= 16;
            }
        }

        for (int i = 0; i < TotRows; i++)
        {
            for (int j = 0; j < TotColumns; j++)
            {
                p = 32 * i + j;
                _mlx.Offset[p] = (short)((eeData[64 + p] & 0xFC00) >> 10);
                if (_mlx.Offset[p] > 31)
                {
                    _mlx.Offset[p] -= 64;
                }

                _mlx.Offset[p] *= (short)(1 << occRemScale);
                _mlx.Offset[p] = (short)(offsetRef + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) +
                                         _mlx.Offset[p]);
            }
        }
    }

    private void ExtractKtaPixelParameters(ushort[] eeData)
    {
        sbyte[] ktaRc = new sbyte[4];
        float[] ktaTemp = new float[768];

        ktaRc[0] = (sbyte)((eeData[54] & 0xFF00) >> 8);
        ktaRc[2] = (sbyte)((eeData[54] & 0x00FF) >> 8);
        ktaRc[1] = (sbyte)((eeData[54] & 0xFF00) >> 8);
        ktaRc[3] = (sbyte)((eeData[54] & 0x00FF) >> 8);

        byte ktaScale1 = (byte)((eeData[56] & 0x00F0) >> 4);
        byte ktaScale2 = (byte)(eeData[56] & 0x000F);

        for (int i = 0; i < TotRows; i++)
        {
            for (int j = 0; j < TotColumns; j++)
            {
                var p = 32 * i + j;
                byte split = (byte)(2 * (p / 32 - p / 64 * 2) + p % 2);
                ktaTemp[p] = (eeData[64 + p] & 0x000E) >> 1;
                if (ktaTemp[p] > 3)
                {
                    ktaTemp[p] -= 8;
                }

                ktaTemp[p] *= 1 << ktaScale2;
                ktaTemp[p] = ktaRc[split] + ktaTemp[p];
                ktaTemp[p] /= (float)Math.Pow(2, ktaScale1);
            }
        }

        float temp = Math.Abs(ktaTemp[0]);
        for (int i = 1; i < TotPixels; i++)
        {
            if (Math.Abs(ktaTemp[i]) > temp)
            {
                temp = Math.Abs(ktaTemp[i]);
            }
        }

        ktaScale1 = 0;
        while (temp < 63.4)
        {
            temp *= 2;
            ktaScale1 = (byte)(ktaScale1 + 1);
        }

        for (int i = 0; i < TotPixels; i++)
        {
            temp = (float)(ktaTemp[i] * Math.Pow(2, ktaScale1));
            if (temp < 0)
            {
                _mlx.Kta[i] = (sbyte)(temp - 0.5);
            }
            else
            {
                _mlx.Kta[i] = (sbyte)(temp + 0.5);
            }
        }

        _mlx.KtaScale = ktaScale1;
    }

    private void ExtractKvPixelParameters(ushort[] eeData)
    {
        sbyte[] kvT = new sbyte[4];
        float[] kvTemp = new float[TotPixels];

        var kvRoCo = (sbyte)((eeData[52] & 0xF000) >> 12);
        if (kvRoCo > 7)
        {
            kvRoCo = (sbyte)(kvRoCo - 16);
        }

        kvT[0] = kvRoCo;

        var kvReCo = (sbyte)((eeData[52] & 0x0F00) >> 8);
        if (kvReCo > 7)
        {
            kvReCo = (sbyte)(kvReCo - 16);
        }

        kvT[2] = kvReCo;

        var kvRoCe = (sbyte)((eeData[52] & 0x00F0) >> 4);
        if (kvRoCe > 7)
        {
            kvRoCe = (sbyte)(kvRoCe - 16);
        }

        kvT[1] = kvRoCe;

        var kvReCe = (sbyte)(eeData[52] & 0x000F);
        if (kvReCe > 7)
        {
            kvReCe = (sbyte)(kvReCe - 16);
        }

        kvT[3] = kvReCe;

        var kvScale = (byte)((eeData[56] & 0x0F00) >> 8);


        for (int i = 0; i < TotRows; i++)
        {
            for (int j = 0; j < TotColumns; j++)
            {
                var p = 32 * i + j;
                var split = (byte)(2 * (p / 32 - p / 64 * 2) + p % 2);
                kvTemp[p] = kvT[split];
                kvTemp[p] /= (float)Math.Pow(2, kvScale);
            }
        }

        var temp = Math.Abs(kvTemp[0]);
        for (int i = 1; i < TotPixels; i++)
        {
            if (Math.Abs(kvTemp[i]) > temp)
            {
                temp = Math.Abs(kvTemp[i]);
            }
        }

        kvScale = 0;
        while (temp < 63.4)
        {
            temp *= 2;
            kvScale = (byte)(kvScale + 1);
        }

        for (int i = 0; i < TotPixels; i++)
        {
            temp = (float)(kvTemp[i] * Math.Pow(2, kvScale));
            if (temp < 0)
            {
                _mlx.Kv[i] = (sbyte)(temp - 0.5);
            }
            else
            {
                _mlx.Kv[i] = (sbyte)(temp + 0.5);
            }
        }

        _mlx.KvScale = kvScale;
    }

    private void ExtractCpParameters(ushort[] eeData)
    {
        float[] alphaSp = new float[2];
        short[] offsetSp = new short[2];

        byte alphaScale = (byte)(((eeData[32] & 0xF000) >> 12) + 27);

        offsetSp[0] = (short)(eeData[58] & 0x03FF);
        if (offsetSp[0] > 511)
        {
            offsetSp[0] = (short)(offsetSp[0] - 1024);
        }

        // offsetSp[1] = (short)((eeData[58] & 0xFC00) >> 10);
        if (offsetSp[1] > 31)
        {
            offsetSp[1] = (short)(offsetSp[1] - 64);
        }

        offsetSp[1] = (short)(offsetSp[1] + offsetSp[0]);

        alphaSp[0] = eeData[57] & 0x03FF;
        if (alphaSp[0] > 511)
        {
            alphaSp[0] -= 1024;
        }

        alphaSp[0] /= (float)Math.Pow(2, alphaScale);

        // alphaSp[1] = (eeData[57] & 0xFC00) >> 10;
        if (alphaSp[1] > 31)
        {
            alphaSp[1] -= 64;
        }

        alphaSp[1] = (1 + alphaSp[1] / 128) * alphaSp[0];

        float cpKta = eeData[59] & 0x00FF;

        byte ktaScale1 = (byte)(((eeData[56] & 0x00F0) >> 4) + 8);
        _mlx.CpKta = (float)(cpKta / Math.Pow(2, ktaScale1));

        float cpKv = (eeData[59] & 0xFF00) >> 8;

        int kvScale = (eeData[56] & 0x0F00) >> 8;
        _mlx.CpKv = (float)(cpKv / Math.Pow(2, kvScale));

        _mlx.CpAlpha[0] = alphaSp[0];
        _mlx.CpAlpha[1] = alphaSp[1];
        _mlx.CpOffset[0] = offsetSp[0];
        _mlx.CpOffset[1] = offsetSp[1];
    }

    private void ExtractCilcParameters(ushort[] eeData)
    {
        float[] ilChessC = new float[3];

        byte calibrationModeEe = (byte)((eeData[10] & 0x0800) >> 4);
        calibrationModeEe = (byte)(calibrationModeEe ^ 0x80);

        ilChessC[0] = (eeData[53] & 0x003F);
        if (ilChessC[0] > 31)
        {
            ilChessC[0] -= 64;
        }

        ilChessC[0] /= 16.0f;

        ilChessC[1] = (eeData[53] & 0x07C0) >> 6;
        if (ilChessC[1] > 15)
        {
            ilChessC[1] -= 32;
        }

        ilChessC[1] /= 2.0f;

        ilChessC[2] = (eeData[53] & 0xF800) >> 11;
        if (ilChessC[2] > 15)
        {
            ilChessC[2] -= 32;
        }

        ilChessC[2] /= 8.0f;

        _mlx.CalibrationModeEe = calibrationModeEe;
        _mlx.IlChessC[0] = ilChessC[0];
        _mlx.IlChessC[1] = ilChessC[1];
        _mlx.IlChessC[2] = ilChessC[2];
    }

    private int ExtractDeviatingPixels(ushort[] eeData)
    {
        ushort pixCnt;
        ushort brokenPixCnt = 0;
        ushort outlierPixCnt = 0;
        int warn = 0;

        for (pixCnt = 0; pixCnt < 5; pixCnt++)
        {
            _mlx.BrokenPixels[pixCnt] = 0xFFFF;
            _mlx.OutlierPixels[pixCnt] = 0xFFFF;
        }

        pixCnt = 0;
        while (pixCnt < TotPixels && brokenPixCnt < 5 && outlierPixCnt < 5)
        {
            if (eeData[pixCnt + 64] == 0)
            {
                _mlx.BrokenPixels[brokenPixCnt] = pixCnt;
                brokenPixCnt = (ushort)(brokenPixCnt + 1);
            }
            else if ((eeData[pixCnt + 64] & 0x0001) != 0)
            {
                _mlx.OutlierPixels[outlierPixCnt] = pixCnt;
                outlierPixCnt = (ushort)(outlierPixCnt + 1);
            }

            pixCnt = (ushort)(pixCnt + 1);
        }

        if (brokenPixCnt > 4)
        {
            warn = -3;
        }
        else if (outlierPixCnt > 4)
        {
            warn = -4;
        }
        else if ((brokenPixCnt + outlierPixCnt) > 4)
        {
            warn = -5;
        }
        else
        {
            int i;
            for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++)
            {
                for (i = pixCnt + 1; i < brokenPixCnt; i++)
                {
                    warn = CheckAdjacentPixels(_mlx.BrokenPixels[pixCnt], _mlx.BrokenPixels[i]);
                    if (warn != 0)
                    {
                        return warn;
                    }
                }
            }

            for (pixCnt = 0; pixCnt < outlierPixCnt; pixCnt++)
            {
                for (i = pixCnt + 1; i < outlierPixCnt; i++)
                {
                    warn = CheckAdjacentPixels(_mlx.OutlierPixels[pixCnt], _mlx.OutlierPixels[i]);
                    if (warn != 0)
                    {
                        return warn;
                    }
                }
            }

            for (pixCnt = 0; pixCnt < brokenPixCnt; pixCnt++)
            {
                for (i = 0; i < outlierPixCnt; i++)
                {
                    warn = CheckAdjacentPixels(_mlx.BrokenPixels[pixCnt], _mlx.BrokenPixels[i]);
                    if (warn != 0)
                    {
                        return warn;
                    }
                }
            }
        }


        return warn;
    }

    private static int CheckAdjacentPixels(ushort pix1, ushort pix2)
    {
        ushort lp1 = (ushort)(pix1 >> 5);
        ushort lp2 = (ushort)(pix2 >> 5);
        ushort cp1 = (ushort)(pix1 - (lp1 << 5));
        ushort cp2 = (ushort)(pix2 - (lp2 << 5));

        var pixPosDif = lp1 - lp2;
        if (pixPosDif is <= -2 or >= 2) return 0;

        pixPosDif = cp1 - cp2;
        if (pixPosDif is > -2 and < 2)
        {
            return -6;
        }

        return 0;
    }

    private async Task<int> GetFrameData(ushort[] frameData)
    {
        int status;
        ushort dataReady = 0;
        ushort statusWord = 0;
        ushort[] auxData = new ushort[64];
        byte cnt;

        while (dataReady == 0)
        {
            statusWord = ReadWordFromRegister(StatusRegister);
            dataReady = (ushort)((statusWord >> 3) & 0b1);
        }

        Console.WriteLine("data ready ! :D");

        while (dataReady != 0)
        {
            status = await WriteInitValueToStatusRegister();
            if (status < 0) // error !
            {
                return status;
            }

            ReadWordsFromRegister(RamStartRegister, frameData);

            // DebugFrame(frameData);

            statusWord = ReadWordFromRegister(StatusRegister);
            dataReady = (ushort)((statusWord >> 3) & 0b1);
        }

        Console.WriteLine($"successfully read frame data from ram ! {DateTime.Now}");

        ReadWordsFromRegister(AuxDataStartAddress, auxData);

        Console.WriteLine("successfully read aux data from aux register !");

        var controlWord = ReadWordFromRegister(ControlRegister);

        frameData[832] = controlWord;
        frameData[833] = (ushort)(statusWord & 0x0001);

        status = ValidateFrameData(frameData);
        if (status != 0)
        {
            Console.WriteLine("frame data validation failed");
            return status;
        }
        
        Console.WriteLine("successfully validated frame data");
        
        status = ValidateAuxData(auxData);
        if (status != 0)
        {
            Console.WriteLine("aux data validation failed");
            return frameData[833];
        }
        
        Console.WriteLine("successfully validated aux data");
        
        for (cnt = 0; cnt < TotAuxData; cnt++)
        {
            frameData[cnt + TotPixels] = auxData[cnt];
        }
        
        Console.WriteLine("successfully assigned aux data to register");

        return frameData[833];
    }

    public async Task<float[]> GetImage()
    {
        float emissivity = 0.95F;
        ushort[] frameData = new ushort[834];
        float[] frame = new float[TotPixels];

        for (int i = 0; i < 2; i++) // first sub-page 0, then sub-page 1
        {
            int status = await GetFrameData(frameData);
            if (status < 0)
            {
                throw new IOException("error while getting data frame");
            }

            var tr = GetTa(frameData) - 8;
            Console.WriteLine($"Tr: {tr}, for calculating pixel To");
            CalculateTo(frameData, emissivity, tr, frame);
        }

        if (frameData.Take(TotPixels).Any(w => w == 0))
        {
            Console.WriteLine("failed to populate both sub-pages");
        }

        return frame;
    }

    // FIXME mlx C version return only zeros
    // public async Task GetImage(float[] result)
    // {
    //     var frameData = new ushort[834]; // array containing all the data we need
    //     var error = await GetFrameData(frameData);
    //     if (error < 0)
    //     {
    //         Console.WriteLine($"failed to get data frame, error code: {error}");
    //         return;
    //     }
    //
    //     float vdd = GetVdd(frameData);
    //     float ta = GetTa(frameData);
    //     float gain = (float)_mlx.GainEe / (short)frameData[778];
    //     float[] irDataCp = new float[2];
    //     float ktaScale = (float)Math.Pow(2, _mlx.KtaScale);
    //     float kvScale = (float)Math.Pow(2, _mlx.KvScale);
    //     byte mode = (byte)((frameData[832] & (1UL << 12)) >> 5);
    //     ushort subPage = frameData[833];
    //
    //     irDataCp[0] = (short)frameData[776] * gain;
    //     irDataCp[1] = (short)frameData[808] * gain;
    //
    //     irDataCp[0] -= _mlx.CpOffset[0] * (1 + _mlx.CpKta * (ta - 25)) * (1 + _mlx.CpKv * (vdd - 3.3f));
    //     if (mode == _mlx.CalibrationModeEe)
    //     {
    //         irDataCp[1] -= _mlx.CpOffset[1] * (1 + _mlx.CpKta * (ta - 25)) *
    //                        (1 + _mlx.CpKv * (vdd - 3.3f));
    //     }
    //     else
    //     {
    //         irDataCp[1] -= (_mlx.CpOffset[1] + _mlx.IlChessC[0]) * (1 + _mlx.CpKta * (ta - 25)) *
    //                        (1 + _mlx.CpKv * (vdd - 3.3f));
    //     }
    //
    //     for (int pixelNumber = 0; pixelNumber < TotPixels; pixelNumber++)
    //     {
    //         int ilPattern = pixelNumber / 32 - (pixelNumber / 64) * 2;
    //         int chessPattern = ilPattern ^ (pixelNumber - (pixelNumber / 2) * 2);
    //         int conversionPattern =
    //             ((pixelNumber + 2) / 4 - (pixelNumber + 3) / 4 + (pixelNumber + 1) / 4 - pixelNumber / 4) *
    //             (1 - 2 * ilPattern);
    //
    //         int pattern = mode == 0 ? ilPattern : chessPattern;
    //
    //         if (pattern != frameData[833]) continue;
    //
    //         float irData = (short)frameData[pixelNumber] * gain;
    //         float kta = _mlx.Kta[pixelNumber] / ktaScale;
    //         float kv = _mlx.Kv[pixelNumber] / kvScale;
    //
    //         irData -= _mlx.Offset[pixelNumber] * (1 + kta * (ta - 25)) * (1 + kv * (vdd - 3.3f));
    //         if (mode != _mlx.CalibrationModeEe)
    //         {
    //             irData += _mlx.IlChessC[2] * (2 * ilPattern - 1) - _mlx.IlChessC[1] * conversionPattern;
    //         }
    //
    //         irData -= _mlx.Tgc * irDataCp[subPage];
    //         float alphaCompensated = _mlx.Alpha[pixelNumber];
    //         float image = irData * alphaCompensated;
    //         result[pixelNumber] = image;
    //     }
    // }

    public string GetRefreshRateToString()
    {
        var word = ReadWordFromRegister(ControlRegister);

        var refreshRate = (word >> 7) & 0b111;

        return RefreshRateToPrettyString(refreshRate);
    }

    public async Task SetRefreshRate(RefreshRate refreshRate)
    {
        ushort controlWord = ReadWordFromRegister(ControlRegister);
        var newRefreshRate = (byte)refreshRate << 7;
        ushort newControlWord = (ushort)(newRefreshRate | (controlWord & 0xFC7F));
        await WriteWordToRegister(ControlRegister, newControlWord);
    }

    private void CalculateTo(ushort[] frameData, float emissivity, float tr, float[] result)
    {
        float[] irDataCp = new float[2]; 
        float[] alphaCorrR = new float[4]; 

        var subPage = frameData[833]; 
        float vdd = GetVdd(frameData); 
        float ta = GetTa(frameData); 

        Console.WriteLine($"Vdd: {vdd}, for calculating pixel To");
        Console.WriteLine($"Ta: {ta}, for calculating pixel To");

        float ta4 = ta + 273.15F; 
        ta4 *= ta4; 
        ta4 *= ta4; 
        float tr4 = tr + 273.15F; 
        tr4 *= tr4; 
        tr4 *= tr4; 
        var taTr = tr4 - (tr4 - ta4) / emissivity; 

        float ktaScale = (float)Math.Pow(2, _mlx.KtaScale); 
        float kvScale = (float)Math.Pow(2, _mlx.KvScale); 
        float alphaScale = (float)Math.Pow(2, _mlx.AlphaScale); 

        alphaCorrR[0] = 1 / (1 + _mlx.KsTo[0] * 40); 
        alphaCorrR[1] = 1; 
        alphaCorrR[2] = 1 + _mlx.KsTo[1] * _mlx.Ct[2]; 
        alphaCorrR[3] = alphaCorrR[2] * (1 + _mlx.KsTo[2] * (_mlx.Ct[3] - _mlx.Ct[2])); 

        //------------------------- Gain calculation -----------------------------------    
        float gain = (float)_mlx.GainEe / (short)frameData[778];

        //------------------------- To calculation -------------------------------------    
        byte mode = (byte)((frameData[832] & 0x1000) >> 5); 

        irDataCp[0] = (short)frameData[776] * gain;
        irDataCp[1] = (short)frameData[808] * gain;

        irDataCp[0] -= (float)(_mlx.CpOffset[0] * (1 + _mlx.CpKta * (ta - 25)) * (1 + _mlx.CpKv * (vdd - 3.3)));
        if (mode == _mlx.CalibrationModeEe)
        {
            irDataCp[1] = (float)(irDataCp[1] - _mlx.CpOffset[1] *
                (1 + _mlx.CpKta * (ta - 25)) * (1 + _mlx.CpKv * (vdd - 3.3)));
        }
        else
        {
            irDataCp[1] = (float)(irDataCp[1] - (_mlx.CpOffset[1] + _mlx.IlChessC[0]) *
                (1 + _mlx.CpKta * (ta - 25)) * (1 + _mlx.CpKv * (vdd - 3.3)));
        }

        for (int pixelNumber = 0; pixelNumber < TotPixels; pixelNumber++)
        {
            sbyte ilPattern = (sbyte)(pixelNumber / 32 - pixelNumber / 64 * 2);
            sbyte chessPattern = (sbyte)(ilPattern ^ (pixelNumber - pixelNumber / 2 * 2));
            sbyte conversionPattern =
                (sbyte)((
                        (pixelNumber + 2) / 4
                        - (pixelNumber + 3) / 4
                        + (pixelNumber + 1) / 4
                        - pixelNumber / 4
                    ) * (1 - 2 * ilPattern)
                );

            var pattern = mode == 0 ? ilPattern : chessPattern;

            if (pattern != frameData[833]) continue;

            float irData = (short)frameData[pixelNumber] * gain;

            var kta = _mlx.Kta[pixelNumber] / ktaScale;
            var kv = _mlx.Kv[pixelNumber] / kvScale;
            irData -= (float)(_mlx.Offset[pixelNumber] * (1 + kta * (ta - 25)) * (1 + kv * (vdd - 3.3)));

            if (mode != _mlx.CalibrationModeEe)
            {
                irData = irData + _mlx.IlChessC[2] * (2 * ilPattern - 1) - _mlx.IlChessC[1] * conversionPattern;
            }

            irData -= _mlx.Tgc * irDataCp[subPage];
            irData /= emissivity;

            float alphaCompensated = (float)(0.000001 * alphaScale / _mlx.Alpha[pixelNumber]);
            alphaCompensated *= 1 + _mlx.KsTa * (ta - 25);

            var sx = alphaCompensated * alphaCompensated * alphaCompensated * (irData + alphaCompensated * taTr);
            sx = (float)(Math.Sqrt(Math.Sqrt(sx)) * _mlx.KsTo[1]);

            var to =
                (float)(Math.Sqrt(Math.Sqrt(irData / (alphaCompensated * (1 - _mlx.KsTo[1] * 273.15) + sx) + taTr)) -
                        273.15);

            sbyte range;
            if (to < _mlx.Ct[1])
            {
                range = 0;
            }
            else if (to < _mlx.Ct[2])
            {
                range = 1;
            }
            else if (to < _mlx.Ct[3])
            {
                range = 2;
            }
            else
            {
                range = 3;
            }

            to = (float)(Math.Sqrt(Math.Sqrt(irData /
                             (alphaCompensated * alphaCorrR[range] *
                              (1 + _mlx.KsTo[range] * (to - _mlx.Ct[range]))) + taTr))
                         - 273.15);

            result[pixelNumber] = to;
        }
    }

    private float GetTa(ushort[] frameData)
    {
        float vdd = GetVdd(frameData);
        short ptat = (short)frameData[800];

        float ptatArt = (short)(ptat / (ptat * _mlx.AlphaPtat + (short)frameData[768]) * Math.Pow(2, 18));

        var ta = (float)(ptatArt / (1 + _mlx.KvPtat * (vdd - 3.3)) - _mlx.VPtat25);
        ta = ta / _mlx.KtPtat + 25;

        return ta;
    }

    // private float GetVdd(ushort[] frameData)
    // {
    //     var resolutionRam = (ushort)((frameData[832] & (~(~0 << 2) << 10)) >> 10);
    //     var resolutionCorrection = (float)(Math.Pow(2, _mlx.ResolutionEe) / Math.Pow(2, resolutionRam));
    //     var vdd = (float)((resolutionCorrection * frameData[810] - _mlx.Vdd25) / _mlx.KVdd + 3.3);
    //
    //     return vdd;
    // }

    private float GetVdd(ushort[] frameData)
    {
        short vdd = (short)frameData[810];

        int resolutionRam = (frameData[832] & 0x0C00) >> 10;
        float resolutionCorrection = (float)(Math.Pow(2, _mlx.ResolutionEe) / Math.Pow(2, resolutionRam));
        return (float)((resolutionCorrection * vdd - _mlx.Vdd25) / _mlx.KVdd + 3.3);
    }

    private static int ValidateFrameData(ushort[] frameData)
    {
        byte line = 0;

        for (int i = 0; i < TotPixels; i += TotColumns)
        {
            if (frameData[i] == 0x7FFF && line % 2 == frameData[833]) return FrameDataError;
            line++;
        }

        return 0;
    }

    private static int ValidateAuxData(ushort[] auxData)
    {
        if (auxData[0] == 0x7FFF) return FrameDataError;

        for (int i = 8; i < TotAuxData; i++)
        {
            if (auxData[i] == 0x7FFF) return FrameDataError;
        }

        return 0;
    }

    private void ReadWordsFromRegister(ushort register, ushort[] words)
    {
        var wordsBuffer = new byte[words.Length * 2];

        ReadFromRegister(register, wordsBuffer);

        // Convert bytes to ushort
        for (int i = 0; i < wordsBuffer.Length; i += 2)
        {
            words[i / 2] = (ushort)((wordsBuffer[i] << 8) | wordsBuffer[i + 1]); // MSB at index 0, LSB at index 1
        }
    }

    private async Task<int> WriteInitValueToStatusRegister()
    {
        ushort initWord = 0x0030;
        int subPage0Check = 0x10;
        int subPage1Check = 0x11;

        await WriteWordToRegister(StatusRegister, initWord);

        ushort dataCheck = ReadWordFromRegister(StatusRegister);
        if (dataCheck == subPage0Check || dataCheck == subPage1Check)
        {
            Console.WriteLine($"page {(dataCheck == subPage0Check ? "0" : "1")} has been measured");
            return dataCheck;
        }

        Console.WriteLine($"data check failed, {dataCheck} != {subPage0Check} && {subPage1Check}");
        return -2;
    }

    private async Task WriteWordToRegister(ushort register, ushort word)
    {
        var cmd = new byte[4];
        cmd[0] = (byte)(register >> 8);
        cmd[1] = (byte)(register & 0xFF);
        cmd[2] = (byte)(word >> 8);
        cmd[3] = (byte)(word & 0xFF);

        _device.Write(cmd);
        
        Console.WriteLine("Wrote: " + string.Join(", ", Array.ConvertAll(cmd, b => "0x" + b.ToString("X"))));
        
        await Task.Delay(1);
    }

    private ushort ReadWordFromRegister(ushort register)
    {
        var wordBuffer = new byte[2];

        ReadFromRegister(register, wordBuffer);

        return (ushort)((wordBuffer[0] << 8) | wordBuffer[1]); // MSB at index 0, LSB at index 1
    }

    private void ReadFromRegister(ushort register, byte[] readBuffer)
    {
        var registerBuffer = new byte[2];
        registerBuffer[0] = (byte)(register >> 8);
        registerBuffer[1] = (byte)(register & 0xFF);

        _device.WriteRead(registerBuffer, readBuffer);
    }

    private static string RefreshRateToPrettyString(int value)
    {
        return value switch
        {
            0b000 => "0.5Hz",
            0b001 => "1Hz",
            0b010 => "2Hz",
            0b011 => "4Hz",
            0b100 => "8Hz",
            0b101 => "16Hz",
            0b110 => "32Hz",
            0b111 => "64Hz",
            _ => "UNKNOWN"
        };
    }
    
    private static void DebugFrame(ushort[] frame)
    {
        Console.WriteLine("Thermal Frame:");
        Console.Write("[");
        for (int i = 0; i < TotRows; i++)
        {
            for (int j = 0; j < TotColumns; j++)
            {
                Console.Write(i * TotColumns + j < TotPixels - 1
                    ? $"{frame[i * TotColumns + j]}, "
                    : $"{frame[i * TotColumns + j]}");
            }
        }

        Console.WriteLine("]");
    }
    
    public enum RefreshRate: byte
    {
        _0_5_Hz = 0b000,
        _1_Hz = 0b001,
        _2_Hz = 0b010,
        _4_Hz = 0b011,
        _8_Hz = 0b100,
        _16_Hz = 0b101,
        _32_Hz = 0b110,
        _64_Hz = 0b111
    }

    private sealed class ParamsMlx
    {
        public short KVdd { get; set; } = -2880;
        public short Vdd25 { get; set; } = -12032;
        public float KvPtat { get; set; } = 0.002930F;
        public float KtPtat { get; set; } = 42.625000F;
        public ushort VPtat25 { get; set; } = 12204;
        public float AlphaPtat { get; set; } = 9.000000F;
        public short GainEe { get; set; } = 5916;
        public float Tgc { get; set; } = 0.000000F;
        public float CpKv { get; set; } = 0.375000F;
        public float CpKta { get; set; } = 0.003784F;
        public byte ResolutionEe { get; set; } = 2;
        public byte CalibrationModeEe { get; set; } = 128;
        public float KsTa { get; set; } = -0.002441F;

        public float[] KsTo { get; } = new float[5]
            { 0.0F, -0.0001983642578125F, -0.0009002685546875F, -0.001495361328125F, -0.0002F };

        public short[] Ct { get; } = new short[5] { -40, 0, 100, 200, 0 };

        public ushort[] Alpha { get; } = new ushort[768]
        {
            24379, 21516, 19599, 18924, 16634, 16145, 15249, 15041, 13726, 13474, 13231, 12845, 12203, 12340, 12203,
            12069, 12003, 12069, 12480, 12551, 12996, 13152, 13726, 14354, 14640, 15144, 16760, 17153, 18604, 19599,
            23343, 27418, 21946, 20513, 18924, 18294, 16145, 15573, 14738, 14544, 13231, 12996, 12770, 12480, 11810,
            11874, 11810, 11684, 11623, 11684, 12069, 12136, 12551, 12696, 13311, 13813, 14169, 14640, 16265, 16509,
            18143, 18924, 21946, 23850, 19775, 18604, 17564, 16634, 14939, 14354, 13557, 13231, 12203, 11938, 11623,
            11441, 10875, 10769, 10716, 10769, 10613, 10822, 10929, 11151, 11441, 11561, 12340, 12770, 13392, 13557,
            15144, 15249, 16634, 17287, 19775, 20513, 19088, 18143, 17020, 16145, 14544, 13988, 13231, 12920, 11874,
            11561, 11265, 11095, 10562, 10461, 10412, 10461, 10266, 10511, 10562, 10822, 11095, 11208, 12003, 12410,
            12996, 13152, 14738, 14838, 16145, 16760, 19088, 19775, 18143, 17287, 15912, 15464, 13474, 13392, 12623,
            12203, 11151, 10984, 10664, 10412, 9896, 9808, 9852, 9852, 9808, 9808, 9941, 10032, 10363, 10562, 11265,
            11441, 11938, 12623, 13813, 14169, 15144, 15797, 18143, 18604, 17705, 16889, 15573, 15041, 13152, 13074,
            12271, 11938, 10929, 10769, 10412, 10171, 9636, 9594, 9636, 9594, 9594, 9594, 9679, 9808, 10171, 10266,
            10984, 11151, 11684, 12271, 13557, 13813, 14838, 15464, 17849, 18143, 16889, 16028, 14738, 14261, 12696,
            12069, 11382, 11323, 10363, 10171, 9721, 9511, 9155, 9079, 9232, 9117, 8932, 9079, 9232, 9430, 9636, 9896,
            10363, 10613, 11208, 11441, 12770, 13152, 14169, 14738, 16889, 17424, 16509, 15797, 14448, 14078, 12480,
            11938, 11208, 11095, 10171, 9986, 9511, 9349, 9005, 8896, 9079, 8932, 8789, 8932, 9079, 9271, 9470, 9721,
            10171, 10412, 11039, 11208, 12551, 12845, 13988, 14448, 16634, 17020, 15912, 15464, 13988, 13392, 12003,
            11561, 10875, 10562, 9896, 9553, 9310, 9117, 8896, 8651, 8617, 8617, 8583, 8583, 8860, 8932, 9117, 9193,
            10032, 10171, 10664, 11039, 12271, 12410, 13641, 14354, 16145, 16634, 15685, 15249, 13813, 13231, 11874,
            11441, 10769, 10461, 9765, 9470, 9232, 9042, 8754, 8550, 8550, 8517, 8484, 8484, 8754, 8824, 9042, 9079,
            9986, 10032, 10562, 10875, 12136, 12203, 13474, 14169, 16028, 16386, 15685, 15041, 13641, 13231, 11810,
            11382, 10769, 10412, 9594, 9349, 9155, 9005, 8583, 8517, 8484, 8387, 8419, 8419, 8685, 8685, 8932, 9155,
            9594, 10032, 10461, 10716, 11747, 12136, 12996, 13813, 15685, 16145, 15685, 14939, 13557, 13152, 11747,
            11323, 10716, 10363, 9553, 9310, 9117, 8968, 8517, 8484, 8451, 8355, 8355, 8355, 8617, 8617, 8896, 9117,
            9553, 9986, 10412, 10664, 11747, 12003, 12920, 13641, 15573, 16145, 15573, 14939, 13474, 12996, 11684,
            11265, 10613, 10363, 9553, 9389, 8968, 8968, 8419, 8387, 8355, 8419, 8292, 8323, 8583, 8617, 8968, 9079,
            9553, 9986, 10511, 10664, 11747, 12069, 13311, 13641, 15464, 16028, 15573, 14939, 13474, 12996, 11684,
            11323, 10613, 10363, 9553, 9389, 9005, 9005, 8451, 8387, 8387, 8419, 8323, 8355, 8583, 8617, 8932, 9079,
            9553, 9986, 10511, 10664, 11747, 12003, 13311, 13641, 15464, 16028, 15685, 15144, 13726, 13074, 11810,
            11501, 10769, 10613, 9765, 9511, 9005, 8896, 8550, 8484, 8484, 8583, 8517, 8517, 8754, 8789, 9042, 9193,
            9808, 9852, 10613, 10984, 11938, 12271, 13557, 13813, 15912, 16145, 15797, 15249, 13813, 13231, 11874,
            11623, 10875, 10664, 9852, 9594, 9079, 8968, 8651, 8583, 8583, 8651, 8617, 8583, 8860, 8860, 9117, 9271,
            9896, 9896, 10716, 11039, 12003, 12340, 13641, 13900, 16028, 16145, 16386, 16028, 14261, 13813, 12410,
            11938, 11151, 11039, 10078, 9852, 9636, 9349, 8932, 8932, 8860, 8824, 8932, 8860, 9155, 9232, 9349, 9594,
            10363, 10412, 10984, 11382, 12410, 12845, 13900, 14544, 16509, 16889, 16634, 16265, 14448, 14078, 12623,
            12136, 11323, 11208, 10218, 9986, 9765, 9470, 9042, 9079, 9005, 8932, 9042, 9005, 9271, 9349, 9470, 9721,
            10461, 10562, 11151, 11561, 12551, 12996, 14078, 14640, 16634, 17153, 17564, 16509, 15144, 14544, 13152,
            12696, 12003, 11623, 10822, 10511, 10078, 9986, 9553, 9511, 9470, 9470, 9430, 9470, 9636, 9852, 9986, 10078,
            10929, 10929, 11747, 11874, 13074, 13813, 14838, 15356, 17424, 17995, 17849, 16889, 15464, 14838, 13392,
            12996, 12271, 11810, 11039, 10716, 10314, 10171, 9765, 9721, 9679, 9679, 9636, 9679, 9852, 10078, 10171,
            10266, 11151, 11151, 12003, 12203, 13311, 14078, 14939, 15573, 17705, 18294, 18924, 17995, 16634, 16028,
            14261, 13988, 12996, 12696, 11810, 11623, 11208, 11039, 10461, 10266, 10314, 10412, 10266, 10124, 10562,
            10664, 10822, 11039, 11874, 11874, 12696, 12920, 14354, 14738, 16145, 16386, 19255, 19954, 19954, 18448,
            17020, 16386, 14640, 14261, 13392, 13074, 12069, 11874, 11501, 11323, 10769, 10562, 10613, 10716, 10562,
            10412, 10822, 10984, 11151, 11382, 12203, 12203, 12996, 13231, 14640, 15144, 16509, 16760, 19775, 21516,
            25809, 21516, 18604, 17849, 15912, 15356, 14640, 14169, 13074, 12770, 12480, 12271, 11747, 11561, 11441,
            11501, 11323, 11323, 11684, 11747, 12069, 12203, 13152, 13474, 14169, 14261, 15797, 16265, 17849, 18294,
            24652, 29241, 30884, 24931, 19255, 18294, 16386, 15797, 15041, 14640, 13474, 13152, 12845, 12623, 12069,
            11874, 11810, 11810, 11623, 11684, 12069, 12069, 12410, 12551, 13474, 13900, 14544, 14738, 16265, 16760,
            18294, 18924, 28857, 34793
        };

        public byte AlphaScale { get; set; } = 10;

        public short[] Offset { get; } = new short[768]
        {
            -40, -48, -39, -53, -42, -51, -41, -57, -44, -55, -43, -61, -47, -57, -45, -65, -46, -61, -46, -66, -45,
            -62, -47, -67, -46, -65, -47, -70, -50, -66, -45, -75, -48, -56, -55, -55, -50, -59, -58, -59, -52, -64,
            -59, -63, -55, -66, -61, -66, -53, -68, -62, -67, -52, -69, -62, -68, -52, -72, -61, -70, -55, -71, -59,
            -75, -43, -50, -40, -55, -45, -52, -43, -59, -47, -57, -45, -63, -49, -60, -48, -66, -48, -62, -49, -68,
            -48, -63, -48, -68, -45, -65, -47, -71, -49, -66, -46, -77, -50, -58, -56, -57, -53, -60, -59, -61, -54,
            -65, -61, -65, -56, -68, -63, -67, -54, -69, -64, -68, -55, -70, -62, -69, -51, -71, -61, -71, -55, -73,
            -60, -77, -41, -50, -42, -55, -43, -52, -43, -58, -44, -57, -46, -63, -48, -60, -48, -66, -45, -62, -49,
            -68, -47, -63, -48, -69, -46, -64, -48, -70, -50, -66, -46, -76, -49, -59, -58, -58, -52, -61, -59, -62,
            -53, -66, -62, -65, -56, -69, -64, -69, -53, -70, -65, -70, -54, -71, -63, -70, -53, -71, -62, -71, -56,
            -74, -60, -78, -44, -50, -42, -56, -45, -54, -45, -59, -48, -57, -48, -65, -51, -62, -48, -67, -50, -63,
            -50, -69, -50, -63, -49, -70, -47, -66, -48, -71, -50, -66, -46, -77, -52, -61, -60, -59, -55, -65, -63,
            -63, -57, -68, -65, -68, -60, -71, -65, -71, -58, -72, -67, -71, -58, -72, -66, -72, -55, -74, -64, -72,
            -57, -74, -62, -79, -45, -50, -43, -57, -46, -54, -46, -61, -48, -59, -48, -65, -49, -62, -50, -68, -49,
            -64, -51, -70, -51, -66, -49, -70, -48, -65, -49, -71, -50, -65, -46, -77, -55, -63, -62, -62, -57, -66,
            -64, -66, -58, -70, -67, -69, -59, -73, -68, -72, -58, -74, -68, -73, -59, -75, -66, -73, -56, -75, -65,
            -74, -57, -74, -63, -79, -42, -50, -44, -56, -45, -54, -46, -60, -48, -59, -50, -65, -49, -61, -52, -69,
            -49, -64, -51, -70, -51, -64, -51, -69, -48, -66, -51, -72, -53, -67, -48, -77, -53, -64, -63, -63, -56,
            -66, -64, -67, -58, -71, -67, -71, -60, -73, -69, -73, -59, -75, -69, -75, -60, -75, -69, -73, -57, -76,
            -67, -75, -61, -77, -64, -81, -47, -51, -46, -58, -49, -58, -48, -62, -51, -60, -51, -66, -53, -63, -53,
            -69, -51, -65, -53, -71, -52, -65, -53, -70, -48, -66, -51, -73, -51, -67, -49, -78, -59, -67, -67, -66,
            -60, -70, -68, -69, -63, -73, -71, -73, -65, -76, -72, -74, -62, -78, -72, -76, -62, -77, -71, -74, -58,
            -78, -69, -76, -60, -77, -67, -82, -50, -51, -48, -59, -50, -55, -49, -62, -51, -60, -53, -69, -55, -64,
            -53, -70, -52, -65, -54, -72, -54, -66, -53, -73, -50, -66, -52, -73, -52, -68, -49, -79, -63, -69, -69,
            -69, -63, -71, -71, -71, -63, -75, -74, -76, -67, -77, -74, -76, -63, -80, -74, -77, -64, -79, -72, -79,
            -60, -79, -71, -78, -61, -79, -68, -84, -49, -51, -49, -60, -50, -57, -51, -64, -52, -61, -54, -69, -54,
            -64, -56, -71, -51, -66, -54, -72, -54, -67, -53, -72, -52, -67, -56, -74, -55, -68, -52, -79, -64, -68,
            -72, -69, -64, -73, -74, -73, -65, -77, -75, -78, -68, -79, -77, -80, -63, -80, -75, -79, -66, -80, -73,
            -79, -62, -81, -74, -80, -65, -81, -71, -86, -54, -54, -52, -62, -54, -57, -52, -65, -54, -62, -55, -70,
            -57, -65, -56, -72, -54, -66, -57, -73, -55, -68, -55, -74, -54, -69, -56, -73, -56, -68, -54, -80, -69,
            -74, -76, -74, -69, -75, -76, -77, -69, -79, -79, -79, -71, -81, -79, -81, -68, -82, -79, -81, -68, -83,
            -77, -83, -65, -83, -77, -81, -67, -83, -74, -87, -56, -55, -54, -62, -55, -58, -54, -66, -56, -63, -55,
            -70, -58, -67, -57, -73, -56, -68, -57, -74, -57, -68, -57, -75, -55, -69, -57, -75, -57, -69, -55, -79,
            -73, -78, -79, -76, -72, -78, -80, -79, -72, -81, -81, -81, -74, -85, -82, -83, -71, -86, -82, -84, -71,
            -85, -80, -85, -69, -87, -80, -84, -69, -86, -76, -88, -55, -55, -55, -64, -56, -60, -56, -68, -56, -63,
            -59, -71, -58, -66, -60, -74, -57, -68, -58, -75, -58, -68, -58, -75, -58, -70, -61, -77, -61, -71, -59,
            -79, -83, -88, -92, -88, -83, -91, -92, -90, -82, -93, -92, -94, -83, -95, -94, -96, -83, -97, -92, -96,
            -82, -96, -92, -94, -80, -98, -92, -97, -82, -96, -85, -99
        };

        public sbyte[] Kta { get; } = new sbyte[768]
        {
            47, 48, 47, 40, 39, 48, 39, 40, 39, 48, 47, 40, 39, 48, 47, 40, 47, 48, 47, 40, 47, 48, 47, 40, 47, 48, 55,
            48, 47, 48, 55, 48, 48, 36, 40, 44, 48, 36, 40, 44, 48, 36, 40, 44, 48, 44, 40, 44, 56, 44, 40, 44, 56, 44,
            40, 44, 56, 44, 40, 44, 56, 44, 48, 44, 39, 48, 47, 40, 39, 48, 47, 40, 39, 48, 47, 40, 47, 48, 39, 40, 47,
            48, 47, 40, 47, 48, 47, 48, 55, 48, 47, 40, 47, 56, 55, 48, 48, 36, 40, 44, 48, 36, 40, 44, 48, 36, 40, 44,
            48, 36, 40, 44, 56, 44, 40, 52, 56, 44, 40, 44, 56, 44, 48, 52, 56, 44, 48, 44, 39, 48, 47, 40, 39, 48, 47,
            40, 47, 48, 47, 40, 47, 48, 47, 40, 47, 48, 47, 40, 47, 48, 47, 40, 55, 48, 47, 48, 47, 56, 55, 48, 48, 36,
            40, 44, 56, 36, 40, 44, 48, 36, 40, 44, 48, 36, 40, 44, 56, 44, 40, 44, 56, 44, 40, 44, 56, 44, 48, 44, 56,
            44, 56, 44, 39, 48, 47, 40, 39, 48, 47, 40, 39, 48, 47, 40, 39, 48, 47, 40, 39, 48, 47, 40, 47, 48, 47, 40,
            47, 48, 55, 48, 47, 56, 55, 48, 48, 36, 40, 44, 48, 36, 40, 44, 48, 36, 40, 52, 48, 44, 40, 44, 56, 44, 40,
            44, 56, 44, 40, 44, 56, 44, 48, 52, 56, 44, 48, 44, 39, 56, 47, 48, 39, 48, 47, 40, 47, 48, 47, 40, 47, 48,
            47, 40, 47, 48, 47, 40, 47, 48, 55, 48, 55, 48, 55, 48, 55, 56, 55, 48, 48, 36, 40, 44, 48, 36, 40, 44, 48,
            36, 40, 44, 48, 44, 40, 44, 56, 44, 40, 44, 56, 44, 48, 44, 56, 44, 48, 52, 56, 44, 48, 52, 47, 48, 47, 40,
            47, 48, 39, 48, 39, 48, 39, 40, 47, 48, 47, 40, 47, 56, 47, 48, 47, 48, 47, 48, 55, 56, 55, 48, 47, 56, 55,
            48, 48, 36, 40, 44, 56, 44, 40, 44, 48, 36, 40, 44, 56, 44, 40, 44, 56, 44, 40, 44, 56, 44, 40, 44, 56, 44,
            48, 44, 56, 44, 48, 44, 47, 56, 39, 40, 47, 56, 47, 40, 39, 48, 47, 40, 47, 48, 47, 40, 47, 48, 47, 48, 47,
            48, 47, 48, 55, 56, 55, 48, 55, 56, 55, 48, 48, 44, 40, 44, 48, 44, 40, 44, 48, 44, 40, 44, 48, 44, 40, 44,
            56, 44, 40, 44, 56, 44, 48, 44, 56, 44, 48, 52, 56, 44, 48, 52, 39, 56, 47, 48, 47, 56, 47, 48, 47, 48, 47,
            40, 47, 56, 47, 40, 47, 56, 47, 40, 47, 48, 55, 48, 55, 56, 55, 48, 55, 56, 55, 48, 48, 44, 40, 44, 48, 44,
            40, 44, 48, 44, 40, 44, 48, 44, 40, 44, 56, 44, 40, 44, 56, 44, 40, 44, 64, 44, 48, 52, 56, 44, 48, 52, 47,
            56, 47, 48, 47, 48, 47, 48, 47, 56, 47, 48, 47, 48, 47, 48, 47, 48, 47, 48, 47, 48, 47, 48, 55, 56, 47, 48,
            47, 56, 55, 48, 56, 44, 40, 44, 56, 44, 40, 44, 48, 44, 40, 44, 56, 44, 40, 44, 56, 44, 40, 44, 56, 44, 40,
            44, 56, 44, 48, 52, 56, 44, 48, 52, 39, 56, 47, 48, 47, 56, 47, 48, 47, 56, 47, 48, 47, 48, 47, 48, 47, 56,
            47, 48, 47, 48, 55, 48, 55, 56, 55, 48, 47, 56, 55, 48, 48, 44, 40, 44, 48, 44, 40, 44, 48, 44, 40, 44, 56,
            44, 40, 44, 56, 44, 40, 44, 56, 44, 48, 44, 56, 44, 48, 52, 56, 44, 48, 52, 47, 56, 47, 48, 47, 56, 47, 48,
            47, 56, 47, 48, 47, 48, 47, 48, 47, 56, 47, 48, 47, 56, 47, 48, 55, 56, 55, 48, 55, 56, 55, 48, 48, 44, 40,
            44, 56, 44, 40, 44, 48, 44, 40, 44, 56, 44, 40, 44, 56, 44, 40, 44, 56, 44, 48, 52, 56, 44, 48, 52, 56, 44,
            48, 52, 47, 56, 47, 56, 47, 56, 47, 48, 47, 56, 47, 48, 47, 56, 47, 48, 47, 56, 47, 48, 47, 56, 55, 48, 47,
            56, 47, 48, 47, 56, 55, 48, 48, 44, 40, 44, 48, 44, 40, 44, 56, 44, 40, 44, 56, 44, 40, 44, 56, 44, 48, 52,
            56, 44, 48, 52, 56, 44, 48, 52, 56, 44, 48, 52
        };

        public byte KtaScale { get; set; } = 13;

        public sbyte[] Kv { get; } = new sbyte[768]
        {
            64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64,
            48, 64, 48, 64, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
            48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64,
            48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
            48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 64, 48, 64, 48, 64, 48, 64,
            48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 48, 48,
            48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
            48, 48, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48,
            64, 48, 64, 48, 64, 48, 64, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
            48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48,
            64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
            48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 64, 48, 64, 48,
            64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64,
            48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
            48, 48, 48, 48, 48, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64,
            48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
            48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64,
            48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 48, 48, 48, 48, 48, 48,
            48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 64,
            48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48,
            64, 48, 64, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
            48, 48, 48, 48, 48, 48, 48, 48, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48,
            64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
            48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 64, 48, 64, 48, 64, 48, 64, 48,
            64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 48, 48, 48,
            48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
            48, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64, 48, 64,
            48, 64, 48, 64, 48, 64, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48,
            48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48
        };

        public byte KvScale { get; set; } = 7;
        public float[] CpAlpha { get; } = new float[2] { 4.540197551250458e-09F, 4.540197551250458e-09F };
        public short[] CpOffset { get; } = new short[2] { -61, -56 };

        public float[] IlChessC { get; } = new float[3]
        {
            0.4375F, 3.5F, 0.125F
        };

        public ushort[] BrokenPixels { get; } = new ushort[5];
        public ushort[] OutlierPixels { get; } = new ushort[5];

        public override string ToString()
        {
            var sb = new StringBuilder();
            sb.AppendLine($"KVdd: {KVdd}");
            sb.AppendLine($"Vdd25: {Vdd25}");
            sb.AppendLine($"KvPtat: {KvPtat}");
            sb.AppendLine($"KtPtat: {KtPtat}");
            sb.AppendLine($"VPtat25: {VPtat25}");
            sb.AppendLine($"AlphaPtat: {AlphaPtat}");
            sb.AppendLine($"GainEe: {GainEe}");
            sb.AppendLine($"Tgc: {Tgc}");
            sb.AppendLine($"CpKv: {CpKv}");
            sb.AppendLine($"CpKta: {CpKta}");
            sb.AppendLine($"ResolutionEe: {ResolutionEe}");
            sb.AppendLine($"CalibrationModeEe: {CalibrationModeEe}");
            sb.AppendLine($"KsTa: {KsTa}");
            sb.AppendLine($"KsTo: [{string.Join(", ", KsTo)}]");
            sb.AppendLine($"Ct: [{string.Join(", ", Ct)}]");
            sb.AppendLine($"Alpha: [{string.Join(", ", Alpha)}]");
            sb.AppendLine($"AlphaScale: {AlphaScale}");
            sb.AppendLine($"Offset: [{string.Join(", ", Offset)}]");
            sb.AppendLine($"Kta: [{string.Join(", ", Kta)}]");
            sb.AppendLine($"KtaScale: {KtaScale}");
            sb.AppendLine($"Kv: [{string.Join(", ", Kv)}]");
            sb.AppendLine($"KvScale: {KvScale}");
            sb.AppendLine($"CpAlpha: [{string.Join(", ", CpAlpha)}]");
            sb.AppendLine($"CpOffset: [{string.Join(", ", CpOffset)}]");
            sb.AppendLine($"IlChessC: [{string.Join(", ", IlChessC)}]");
            sb.AppendLine($"BrokenPixels: [{string.Join(", ", BrokenPixels)}]");
            sb.AppendLine($"OutlierPixels: [{string.Join(", ", OutlierPixels)}]");
            return sb.ToString();
        }
    }

    public void Dispose()
    {
        _device.Dispose();
    }
}