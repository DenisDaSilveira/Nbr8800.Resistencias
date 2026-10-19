using System;
using System.Collections.Generic;
using System.Linq;

namespace Nbr8800.Resistencias
{
    /// <summary>Resistências de cálculo conforme ABNT NBR 8800:2024, versão corrigida 25/02/2025.</summary>
    public sealed class CalculadoraNBR8800
    {
        public ResultadoResistencia Tracao(SecaoMetalica s, MaterialAco m, double ct = 1.0)
        {
            Validar(s, m);
            if (ct <= 0 || ct > 1) throw new ArgumentOutOfRangeException(nameof(ct));
            double an = s.An > 0 ? s.An : s.Ag;
            double ny = s.Ag * m.Fy / m.GammaA1;
            double nu = ct * an * m.Fu / m.GammaA2;
            var r = Novo(Math.Min(ny, nu), ny <= nu ? EstadoLimite.EscoamentoSecaoBruta : EstadoLimite.RupturaSecaoLiquida);
            r.Valores["Nt,Rd_escoamento"] = ny;
            r.Valores["Nt,Rd_ruptura"] = nu;
            r.Memoria.Add("5.2.2: Nt,Rd = min(Ag*fy/gamma_a1; Ct*An*fu/gamma_a2).");
            return r;
        }

        public ResultadoResistencia Compressao(SecaoMetalica s, MaterialAco m, ComprimentosFlambagem l)
        {
            Validar(s, m);
            if (l == null || l.Lx <= 0 || l.Ly <= 0 || l.Lz <= 0) throw new ArgumentException("Lx, Ly e Lz devem ser positivos.");
            double nex = Math.PI * Math.PI * m.E * s.Ix / (l.Lx * l.Lx);
            double ney = Math.PI * Math.PI * m.E * s.Iy / (l.Ly * l.Ly);
            double nez = (Math.PI * Math.PI * m.E * s.Cw / (l.Lz * l.Lz) + m.G * s.J) / s.R0Quadrado;
            double ne;
            EstadoLimite modo;

            if (s.Tipo == TipoSecao.GenericaMonossimetrica || s.Tipo == TipoSecao.IouHMonossimetrica || s.Tipo == TipoSecao.T || s.Tipo == TipoSecao.U)
            {
                bool simetriaY = Math.Abs(s.X0) < 1e-12;
                double nef = simetriaY ? nex : ney;
                double nePlano = simetriaY ? ney : nex;
                double desloc = simetriaY ? s.Y0 : s.X0;
                double q = 1.0 - desloc * desloc / s.R0Quadrado;
                if (q <= 0) throw new ArgumentException("Centro de cisalhamento inválido para flexo-torção.");
                double neft = (nePlano + nez) / (2 * q) * (1 - Math.Sqrt(1 - 4 * nePlano * nez * q / Math.Pow(nePlano + nez, 2)));
                ne = Math.Min(nef, neft);
                modo = nef <= neft ? (simetriaY ? EstadoLimite.FlexaoX : EstadoLimite.FlexaoY) : EstadoLimite.FlexoTorcao;
            }
            else if (s.Tipo == TipoSecao.GenericaAssimetrica || s.Tipo == TipoSecao.Cantoneira)
            {
                ne = MenorRaizPositivaCubicBuckling(nex, ney, nez, s.X0, s.Y0, s.R0Quadrado);
                modo = EstadoLimite.FlexoTorcao;
            }
            else
            {
                ne = Math.Min(nex, Math.Min(ney, nez));
                modo = ne == nex ? EstadoLimite.FlexaoX : ne == ney ? EstadoLimite.FlexaoY : EstadoLimite.Torcao;
            }

            double lambda0 = Math.Sqrt(s.Ag * m.Fy / ne);
            double chi = lambda0 <= 1.5 ? Math.Pow(0.658, lambda0 * lambda0) : 0.877 / (lambda0 * lambda0);
            double aef = AreaEfetivaCompressao(s, m, chi);
            double nrd = chi * aef * m.Fy / m.GammaA1;
            var r = Novo(nrd, aef < s.Ag - 1e-8 ? EstadoLimite.FlambagemLocal : modo);
            r.Valores["Nex"] = nex; r.Valores["Ney"] = ney; r.Valores["Nez"] = nez;
            r.Valores["Ne"] = ne; r.Valores["lambda0"] = lambda0; r.Valores["chi"] = chi; r.Valores["Aef"] = aef;
            r.Memoria.Add("5.3.2/5.3.3: Nc,Rd = chi*Aef*fy/gamma_a1.");
            if (Math.Max(l.Lx / s.Rx, l.Ly / s.Ry) > 200) r.Avisos.Add("Índice de esbeltez superior ao valor recomendado de 200 (5.3.7.1).");
            return r;
        }

        public ResultadoClassificacao ClassificarFlexao(SecaoMetalica s, MaterialAco m, EixoFlexao eixo)
        {
            Validar(s, m);
            double raiz = Math.Sqrt(m.E / m.Fy);
            var valores = new List<int>(); // 0 compacta, 1 semicompacta, 2 esbelta
            if (s.Tipo == TipoSecao.IouHDuplaSimetria || s.Tipo == TipoSecao.IouHMonossimetrica || s.Tipo == TipoSecao.U)
            {
                double bMesa = s.Tipo == TipoSecao.U ? s.Bf : s.Bf / 2.0;
                double lm = bMesa / s.Tf;
                double lp = 0.38 * raiz;
                double sr = 0.30 * m.Fy;
                double lr = s.Fabricacao == FabricacaoPerfil.Laminado ? 0.83 * Math.Sqrt(m.E / (m.Fy - sr)) : 0.95 * Math.Sqrt(m.E / (m.Fy - sr));
                valores.Add(Faixa(lm, lp, lr));
            }
            if (eixo == EixoFlexao.X && (s.Tipo == TipoSecao.IouHDuplaSimetria || s.Tipo == TipoSecao.IouHMonossimetrica || s.Tipo == TipoSecao.U))
                valores.Add(Faixa(s.H / s.Tw, 3.76 * raiz, 5.70 * raiz));
            if (s.Tipo == TipoSecao.TubularCircular)
                valores.Add(Faixa(s.Diametro / s.EspessuraTubo, 0.07 * m.E / m.Fy, 0.31 * m.E / m.Fy));
            if (!valores.Any()) throw new NotSupportedException("Classificação automática ainda não definida para este tipo/eixo; use os estados-limites específicos.");
            int pior = valores.Max();
            var r = new ResultadoClassificacao { Classe = pior == 0 ? ClasseSecao.Compacta : pior == 1 ? ClasseSecao.Semicompacta : ClasseSecao.Esbelta };
            r.Memoria.Add("Classificação pela pior relação largura/espessura conforme 5.1.2 e Anexo D.");
            return r;
        }

        public ResultadoResistencia Flexao(SecaoMetalica s, MaterialAco m, ParametrosFlexao p)
        {
            Validar(s, m);
            if (p == null) throw new ArgumentNullException(nameof(p));
            double w = p.Eixo == EixoFlexao.X ? s.Wx : s.Wy;
            double z = p.Eixo == EixoFlexao.X ? s.Zx : s.Zy;
            double mpl = z * m.Fy;
            var limites = new Dictionary<EstadoLimite, double>();

            if (s.Tipo == TipoSecao.TubularCircular)
            {
                double lambda = s.Diametro / s.EspessuraTubo;
                double lp = 0.07 * m.E / m.Fy, lr = 0.31 * m.E / m.Fy;
                double mn = lambda <= lp ? mpl : lambda <= lr ? (0.021 * m.E / lambda + m.Fy) * w : 0.33 * m.E * w / lambda;
                limites[EstadoLimite.FlambagemLocal] = mn / m.GammaA1;
            }
            else
            {
                limites[EstadoLimite.EscoamentoSecaoBruta] = mpl / m.GammaA1;
                AdicionarFlambagemLocal(s, m, p, w, mpl, limites);
                AdicionarFlt(s, m, p, w, mpl, limites);
            }

            var gov = limites.OrderBy(x => x.Value).First();
            var r = Novo(gov.Value, gov.Key);
            foreach (var x in limites) r.Valores[x.Key.ToString()] = x.Value;
            r.Memoria.Add("5.4.2 e Anexo D: MRd é o menor valor dos estados-limites aplicáveis.");
            if ((s.Tipo == TipoSecao.IouHDuplaSimetria || s.Tipo == TipoSecao.IouHMonossimetrica) && s.H / s.Tw > 5.70 * Math.Sqrt(m.E / m.Fy))
                r.Avisos.Add("Alma esbelta: dimensionar também pelo Anexo E; o resultado do Anexo D não é suficiente.");
            return r;
        }

        //public ResultadoResistencia Cortante(SecaoMetalica s, MaterialAco m, ParametrosCortante p)
        //{
        //    Validar(s, m);
        //    if (p == null) throw new ArgumentNullException(nameof(p));
        //    if (s.Tipo == TipoSecao.TubularCircular) return CortanteTuboCircular(s, m, p);

        //    double h, t, aw, kv;
        //    bool eixoMaior = p.EixoFlexao == EixoFlexao.X;
        //    switch (s.Tipo)
        //    {
        //        case TipoSecao.IouHDuplaSimetria:
        //        case TipoSecao.IouHMonossimetrica:
        //        case TipoSecao.U:
        //            if (eixoMaior) { h = s.H; t = s.Tw; aw = s.D * s.Tw; kv = Kv(h, p.EspacamentoEnrijecedores); }
        //            else { h = s.Tipo == TipoSecao.U ? s.Bf : s.Bf / 2; t = s.Tf; aw = 2 * s.Bf * s.Tf; kv = 1.2; }
        //            break;
        //        case TipoSecao.TubularRetangular:
        //        case TipoSecao.Caixao:
        //            h = s.H; t = s.Tw; aw = 2 * h * t; kv = 5.0; break;
        //        case TipoSecao.T:
        //            h = s.D; t = s.Tw; aw = s.D * s.Tw; kv = 1.2; break;
        //        case TipoSecao.DuasCantoneirasEmT:
        //            h = s.D; t = s.Tw; aw = 2 * h * t; kv = 1.2; break;
        //        default: throw new NotSupportedException("Cortante não implementado para este tipo de seção.");
        //    }
        //    double lambda = h / t;
        //    double lp = 1.10 * Math.Sqrt(kv * m.E / m.Fy);
        //    double lr = 1.37 * Math.Sqrt(kv * m.E / m.Fy);
        //    double vpl = 0.60 * aw * m.Fy;
        //    double fator = lambda <= lp ? 1 : lambda <= lr ? lp / lambda : 1.24 * Math.Pow(lp / lambda, 2);
        //    var estado = lambda <= lp ? EstadoLimite.PlastificacaoCisalhamento : EstadoLimite.FlambagemCisalhamento;
        //    var r = Novo(fator * vpl / m.GammaA1, estado);
        //    r.Valores["lambda"] = lambda; r.Valores["lambdaP"] = lp; r.Valores["lambdaR"] = lr; r.Valores["kv"] = kv; r.Valores["Vpl"] = vpl;
        //    r.Memoria.Add("5.4.3.1: VRd por plastificação ou instabilidade por cisalhamento.");
        //    return r;
        //}



        public ResultadoResistencia Cortante(
            SecaoMetalica s,
            MaterialAco m,
            ParametrosCortante p)
        {
            Validar(s, m);

            if (p == null)
                throw new ArgumentNullException(nameof(p));

            if (s.Tipo == TipoSecao.TubularCircular)
                return CortanteTuboCircular(s, m, p);

            double h;
            double t;
            double aw;
            double kv;

            bool eixoX =
                p.EixoFlexao == EixoFlexao.X;

            switch (s.Tipo)
            {
                case TipoSecao.IouHDuplaSimetria:
                case TipoSecao.IouHMonossimetrica:
                case TipoSecao.U:
                    {
                        if (eixoX)
                        {
                            h = s.H;
                            t = s.Tw;
                            aw = s.D * s.Tw;

                            kv = Kv(
                                h,
                                p.EspacamentoEnrijecedores);
                        }
                        else
                        {
                            h = s.Tipo == TipoSecao.U
                                ? s.Bf
                                : s.Bf / 2.0;

                            t = s.Tf;
                            aw = 2.0 * s.Bf * s.Tf;
                            kv = 1.20;
                        }

                        break;
                    }

                case TipoSecao.GenericaDuplaSimetria:
                    {
                        /*
                         * Para seção genérica duplamente simétrica, utiliza-se
                         * preferencialmente a área de cisalhamento cadastrada.
                         *
                         * Ajuste os nomes AreaCisalhamentoX e AreaCisalhamentoY
                         * conforme as propriedades existentes em SecaoMetalica.
                         */

                        if (eixoX)
                        {
                            h = s.H;
                            t = s.Tw;

                            aw = s.AreaCisalhamentoX > 0.0
                                ? s.AreaCisalhamentoX
                                : s.D * s.Tw;

                            kv = Kv(
                                h,
                                p.EspacamentoEnrijecedores);
                        }
                        else
                        {
                            h = s.Bf / 2.0;
                            t = s.Tf;

                            aw = s.AreaCisalhamentoY > 0.0
                                ? s.AreaCisalhamentoY
                                : 2.0 * s.Bf * s.Tf;

                            kv = 1.20;
                        }

                        break;
                    }

                case TipoSecao.TubularRetangular:
                case TipoSecao.Caixao:
                    {
                        h = s.H;
                        t = s.Tw;
                        aw = 2.0 * h * t;
                        kv = 5.0;
                        break;
                    }

                case TipoSecao.T:
                    {
                        h = s.D;
                        t = s.Tw;
                        aw = s.D * s.Tw;
                        kv = 1.20;
                        break;
                    }

                case TipoSecao.Cantoneira:
                    {
                        t = s.Tw;
                        h = eixoX
                            ? s.H
                            : s.Bf;

                        aw = Math.Max(h - t, 0.0) * t;
                        kv = 1.20;
                        break;
                    }

                case TipoSecao.DuasCantoneirasEmT:
                    {
                        h = s.D;
                        t = s.Tw;
                        aw = 2.0 * h * t;
                        kv = 1.20;
                        break;
                    }

                default:
                    throw new NotSupportedException(
                        $"Cortante não implementado para o tipo de seção {s.Tipo}.");
            }

            if (h <= 0.0)
            {
                throw new InvalidOperationException(
                    $"Altura resistente ao cortante inválida para a seção {s.Tipo}: h = {h}.");
            }

            if (t <= 0.0)
            {
                throw new InvalidOperationException(
                    $"Espessura resistente ao cortante inválida para a seção {s.Tipo}: t = {t}.");
            }

            if (aw <= 0.0)
            {
                throw new InvalidOperationException(
                    $"Área resistente ao cortante inválida para a seção {s.Tipo}: Aw = {aw}.");
            }

            if (kv <= 0.0)
            {
                throw new InvalidOperationException(
                    $"Coeficiente de flambagem por cisalhamento inválido: kv = {kv}.");
            }

            double lambda =
                h / t;

            double lambdaP =
                1.10 * Math.Sqrt(
                    kv * m.E / m.Fy);

            double lambdaR =
                1.37 * Math.Sqrt(
                    kv * m.E / m.Fy);

            double vpl =
                0.60 * aw * m.Fy;

            double fator;
            EstadoLimite estado;

            if (lambda <= lambdaP)
            {
                fator = 1.0;
                estado =
                    EstadoLimite.PlastificacaoCisalhamento;
            }
            else if (lambda <= lambdaR)
            {
                fator =
                    lambdaP / lambda;

                estado =
                    EstadoLimite.FlambagemCisalhamentoTorcao;
            }
            else
            {
                fator =
                    1.24 *
                    Math.Pow(
                        lambdaP / lambda,
                        2.0);

                estado =
                    EstadoLimite.FlambagemCisalhamentoTorcao;
            }

            ResultadoResistencia resultado =
                Novo(
                    fator * vpl / m.GammaA1,
                    estado);

            resultado.Valores["h"] = h;
            resultado.Valores["t"] = t;
            resultado.Valores["Aw"] = aw;
            resultado.Valores["lambda"] = lambda;
            resultado.Valores["lambdaP"] = lambdaP;
            resultado.Valores["lambdaR"] = lambdaR;
            resultado.Valores["kv"] = kv;
            resultado.Valores["Vpl"] = vpl;
            resultado.Valores["fator"] = fator;

            resultado.Memoria.Add(
                "5.4.3.1: VRd por plastificação ou instabilidade por cisalhamento.");

            if (s.Tipo == TipoSecao.GenericaDuplaSimetria)
            {
                resultado.Memoria.Add(
                    eixoX
                        ? "Seção genérica duplamente simétrica: cortante no eixo X."
                        : "Seção genérica duplamente simétrica: cortante no eixo Y.");

                if (eixoX && s.AreaCisalhamentoX <= 0.0)
                {
                    resultado.Avisos.Add(
                        "AreaCisalhamentoX não informada. Adotado Aw = D × Tw.");
                }

                if (!eixoX && s.AreaCisalhamentoY <= 0.0)
                {
                    resultado.Avisos.Add(
                        "AreaCisalhamentoY não informada. Adotado Aw = 2 × Bf × Tf.");
                }
            }

            return resultado;
        }





        /// <summary>
        /// Resistência de cálculo à torção de seções tubulares fechadas, pelo
        /// fluxo de cisalhamento de Bredt. Para perfis abertos a verificação
        /// plástica à torção requer propriedades de empenamento/seção efetiva
        /// específicas e, por segurança, não é liberada por este método.
        /// Unidades: N e mm.
        /// </summary>
        public ResultadoResistencia Torcao(
    SecaoMetalica s,
    MaterialAco m)
        {
            Validar(s, m);

            double am;
            double t;

            switch (s.Tipo)
            {
                case TipoSecao.TubularRetangular:
                case TipoSecao.Caixao:
                    {
                        if (s.Bf <= 0.0 ||
                            s.H <= 0.0 ||
                            s.Tw <= 0.0 ||
                            s.Tf <= 0.0)
                        {
                            throw new ArgumentException(
                                "Geometria incompleta para torção de seção fechada.");
                        }

                        /*
                         * Dimensões da linha média das paredes.
                         */
                        double larguraMedia =
                            s.Bf - s.Tw;

                        double alturaMedia =
                            s.H - s.Tf;

                        if (larguraMedia <= 0.0 ||
                            alturaMedia <= 0.0)
                        {
                            throw new ArgumentException(
                                "Dimensões da linha média inválidas para a seção fechada.");
                        }

                        am =
                            larguraMedia *
                            alturaMedia;

                        t =
                            Math.Min(
                                s.Tw,
                                s.Tf);

                        break;
                    }

                case TipoSecao.TubularCircular:
                    {
                        if (s.Diametro <= 0.0 ||
                            s.EspessuraTubo <= 0.0)
                        {
                            throw new ArgumentException(
                                "Geometria incompleta para torção de tubo circular.");
                        }

                        if (2.0 * s.EspessuraTubo >= s.Diametro)
                        {
                            throw new ArgumentException(
                                "A espessura do tubo é incompatível com o diâmetro.");
                        }

                        double diametroMedio =
                            s.Diametro -
                            s.EspessuraTubo;

                        am =
                            Math.PI *
                            diametroMedio *
                            diametroMedio /
                            4.0;

                        t =
                            s.EspessuraTubo;

                        break;
                    }

                default:
                    throw new NotSupportedException(
                        $"A seção aberta {s.Tipo} exige análise de empenamento. " +
                        "Utilize a sobrecarga Torcao(s, m, p), informando J, Cw, " +
                        "ωmax, bimomento BSd e a parcela torsional de Saint-Venant.");
            }

            if (am <= 0.0 || t <= 0.0)
            {
                throw new ArgumentException(
                    "Área média ou espessura inválida para torção.");
            }

            double tauRd =
                0.60 *
                m.Fy /
                m.GammaA1;

            double trd =
                2.0 *
                am *
                t *
                tauRd;

            ResultadoResistencia resultado =
                Novo(
                    trd,
                    EstadoLimite.Torcao);

            resultado.Valores["Am"] =
                am;

            resultado.Valores["tMin"] =
                t;

            resultado.Valores["tauRd"] =
                tauRd;

            resultado.Valores["TRd"] =
                trd;

            resultado.Memoria.Add(
                "Seção fechada unicelular: fluxo de cisalhamento de Bredt.");

            resultado.Memoria.Add(
                "TRd = 2 · Am · tmin · τRd.");

            resultado.Memoria.Add(
                "τRd = 0,60 · fy / γa1.");

            return resultado;
        }

        private static void AdicionarFlambagemLocal(SecaoMetalica s, MaterialAco m, ParametrosFlexao p, double w, double mpl, IDictionary<EstadoLimite, double> d)
        {
            double raiz = Math.Sqrt(m.E / m.Fy);
            if (s.Tipo == TipoSecao.IouHDuplaSimetria || s.Tipo == TipoSecao.IouHMonossimetrica || s.Tipo == TipoSecao.U)
            {
                double b = s.Tipo == TipoSecao.U ? s.Bf : s.Bf / 2;
                double lambda = b / s.Tf, lp = 0.38 * raiz, sr = .30 * m.Fy;
                double lr = s.Fabricacao == FabricacaoPerfil.Laminado ? .83 * Math.Sqrt(m.E / (m.Fy - sr)) : .95 * Math.Sqrt(m.E / (m.Fy - sr));
                double mr = (m.Fy - sr) * w;
                double mcr = (s.Fabricacao == FabricacaoPerfil.Laminado ? .69 : .90) * m.E * w / (lambda * lambda);
                d[EstadoLimite.FLM] = Interpolar(mpl, mr, mcr, lambda, lp, lr) / m.GammaA1;
                if (p.Eixo == EixoFlexao.X)
                {
                    lambda = s.H / s.Tw; lp = 3.76 * raiz; lr = 5.70 * raiz;
                    if (lambda <= lr) d[EstadoLimite.FLA] = Interpolar(mpl, m.Fy * w, m.Fy * w, lambda, lp, lr) / m.GammaA1;
                }
            }
            else if (s.Tipo == TipoSecao.TubularRetangular || s.Tipo == TipoSecao.Caixao)
            {
                double lambda = s.Bf / s.Tf, lp = 1.12 * raiz;
                double lr = (s.Tipo == TipoSecao.TubularRetangular ? 1.40 : 1.49) * raiz;
                double wef = p.Wef > 0 ? p.Wef : w;
                double mn = lambda <= lp ? mpl : lambda <= lr ? mpl - (mpl - m.Fy * wef) * (3.57 * lambda * Math.Sqrt(m.Fy / m.E) - 4.0) : m.Fy * wef;
                d[EstadoLimite.FLM] = mn / m.GammaA1;
            }
        }

        private static void AdicionarFlt(SecaoMetalica s, MaterialAco m, ParametrosFlexao p, double w, double mpl, IDictionary<EstadoLimite, double> d)
        {
            if (p.Eixo != EixoFlexao.X || p.Lb <= 0) return;
            // Seções tubulares/caixão fechadas não sofrem a FLT típica dos
            // perfis abertos; suas instabilidades locais são tratadas à parte.
            if (s.Tipo != TipoSecao.IouHDuplaSimetria && s.Tipo != TipoSecao.U && s.Tipo != TipoSecao.SolidaRetangular) return;
            double lambda = p.Lb / s.Ry;
            double lp = 1.76 * Math.Sqrt(m.E / 10 / m.Fy);
            double mr = (m.Fy - .30 * m.Fy) * w;
            double mcr;
            if (p.McrAlternativo > 0) mcr = p.McrAlternativo;
            else if ((s.Tipo == TipoSecao.IouHDuplaSimetria || s.Tipo == TipoSecao.U) && s.Cw > 0)
                mcr = p.Cb * Math.PI * Math.PI * m.E * s.Iy / (p.Lb * p.Lb) * Math.Sqrt(s.Cw / s.Iy * (1 + .039 * s.J * p.Lb * p.Lb / s.Cw));
            else
                mcr = 2 * m.E * p.Cb * Math.Sqrt(s.J * s.Ag) / lambda;
            double lambdaLt = Math.Sqrt(mpl / mcr);
            double chi = lambdaLt <= .4 ? 1 : lambdaLt <= 1.4 ? 1 - .49 * (lambdaLt - .4) : 1 / (lambdaLt * lambdaLt);
            d[EstadoLimite.FLT] = Math.Min(mpl, chi * mpl) / m.GammaA1;
        }

        private static double AreaEfetivaCompressao(SecaoMetalica s, MaterialAco m, double chi)
        {
            if (s.Tipo == TipoSecao.TubularCircular)
            {
                double dt = s.Diametro / s.EspessuraTubo;
                if (dt <= .11 * m.E / m.Fy) return s.Ag;
                if (dt > .45 * m.E / m.Fy) throw new ArgumentException("D/t excede 0,45 E/fy (5.3.4.3).");
                return (.038 / dt * m.E / m.Fy + 2.0 / 3.0) * s.Ag;
            }
            double perda = 0;
            foreach (var e in s.ElementosComprimidos)
            {
                double lim = LimiteCompressao(e, m);
                double lambda = e.B / e.T;
                if (lambda <= lim / Math.Sqrt(chi)) continue;
                double c1 = e.Classe == ClasseElemento.AL ? .22 : e.Classe == ClasseElemento.ParedeTubularRetangular ? .20 : .18;
                double c2 = e.Classe == ClasseElemento.AL ? 1.49 : e.Classe == ClasseElemento.ParedeTubularRetangular ? 1.38 : 1.31;
                double sigmaEl = Math.Pow(c2 * lim / lambda, 2) * m.Fy;
                double raiz = Math.Sqrt(sigmaEl / (chi * m.Fy));
                double bef = e.B * (1 - c1 * raiz) * raiz;
                perda += Math.Max(0, e.B - bef) * e.T * e.Quantidade;
            }
            return Math.Max(0, s.Ag - perda);
        }

        private static double LimiteCompressao(ElementoComprimido e, MaterialAco m)
        {
            double q = Math.Sqrt(m.E / m.Fy);
            switch (e.Grupo)
            {
                case GrupoElementoComprimido.Grupo1: return 1.40 * q;
                case GrupoElementoComprimido.Grupo2: return 1.49 * q;
                case GrupoElementoComprimido.Grupo3: return .45 * q;
                case GrupoElementoComprimido.Grupo4: return .56 * q;
                case GrupoElementoComprimido.Grupo5:
                    double kc = e.Kc > 0 ? e.Kc : Math.Max(.35, Math.Min(.76, 4 / Math.Sqrt(e.H / e.Tw)));
                    return .64 * Math.Sqrt(m.E / (m.Fy / kc));
                case GrupoElementoComprimido.Grupo6: return .75 * q;
                default: throw new ArgumentOutOfRangeException();
            }
        }

        private static ResultadoResistencia CortanteTuboCircular(SecaoMetalica s, MaterialAco m, ParametrosCortante p)
        {
            if (p.Lv <= 0) throw new ArgumentException("Lv deve ser positivo para tubo circular.");
            double dt = s.Diametro / s.EspessuraTubo;
            double a = 1.60 * m.E / (Math.Sqrt(p.Lv / s.Diametro) * Math.Pow(dt, 1.25));
            double b = .78 * m.E / Math.Pow(dt, 1.5);
            double tau = Math.Min(.60 * m.Fy, Math.Max(a, b));
            var r = Novo(.5 * tau * s.Ag / m.GammaA1, EstadoLimite.FlambagemCisalhamentoTorcao);
            r.Valores["tauCr"] = tau;
            r.Memoria.Add("5.4.3.6: cortante de seção tubular circular.");
            return r;
        }

        private static double Kv(double h, double a) { return a <= 0 || a / h > 3 ? 5.34 : 5.0 + 5.0 / Math.Pow(a / h, 2); }
        private static int Faixa(double l, double lp, double lr) { return l <= lp ? 0 : l <= lr ? 1 : 2; }
        private static double Interpolar(double mp, double mr, double mcr, double l, double lp, double lr) { return l <= lp ? mp : l <= lr ? mp - (mp - mr) * (l - lp) / (lr - lp) : mcr; }
        private static ResultadoResistencia Novo(double v, EstadoLimite e)
        {
            if (double.IsNaN(v) || double.IsInfinity(v) || v < 0)
                throw new InvalidOperationException("Resistência não finita; verifique Zx/Zy, J, Cw e dimensões da seção.");
            return new ResultadoResistencia { Valor = v, Governante = e };
        }


        public ResultadoResistencia Torcao(
    SecaoMetalica s,
    MaterialAco m,
    ParametrosTorcao p)
        {
            Validar(s, m);

            if (p == null)
                throw new ArgumentNullException(nameof(p));

            /*
             * Não executar análise torsional quando não houver
             * momento torsor solicitante.
             *
             * Ajuste a tolerância conforme a unidade utilizada:
             * kN.m, N.mm, tf.m etc.
             */
            const double toleranciaTorcao = 1.0e-9;

            if (Math.Abs(p.TSd) <= toleranciaTorcao)
            {
                ResultadoResistencia resultado =
                    Novo(
                        double.PositiveInfinity,
                        EstadoLimite.Torcao);

                resultado.Valores["TSd"] = 0.0;
                resultado.Valores["TRd"] = double.PositiveInfinity;
                resultado.Valores["indiceTorcao"] = 0.0;

                resultado.Memoria.Add(
                    "Verificação à torção dispensada: momento torsor solicitante TSd nulo.");

                return resultado;
            }

            switch (s.Tipo)
            {
                /*
                 * Seções fechadas:
                 * usa a rotina existente baseada no fluxo de Bredt.
                 */
                case TipoSecao.TubularRetangular:
                case TipoSecao.Caixao:
                case TipoSecao.TubularCircular:
                    return TorcaoSecaoFechada(s, m, p);

                /*
                 * Seções abertas:
                 * exige análise específica de Saint-Venant e empenamento.
                 */
                case TipoSecao.IouHDuplaSimetria:
                case TipoSecao.IouHMonossimetrica:
                case TipoSecao.U:
                case TipoSecao.T:
                case TipoSecao.Cantoneira:
                case TipoSecao.DuasCantoneirasEmT:
                case TipoSecao.GenericaDuplaSimetria:
                    return TorcaoSecaoAberta(
                        s,
                        m,
                        p);

                default:
                    throw new NotSupportedException(
                        $"Verificação à torção não implementada para a seção {s.Tipo}.");
            }
        }


        private ResultadoResistencia TorcaoSecaoFechada(
    SecaoMetalica s,
    MaterialAco m,
    ParametrosTorcao p)
        {
            if (s == null)
                throw new ArgumentNullException(nameof(s));

            if (m == null)
                throw new ArgumentNullException(nameof(m));

            if (p == null)
                throw new ArgumentNullException(nameof(p));

            if (Math.Abs(p.TSd) <= 0.0)
            {
                ResultadoResistencia resultadoNulo =
                    Novo(
                        double.PositiveInfinity,
                        EstadoLimite.Torcao);

                resultadoNulo.Valores["TSd"] = 0.0;
                resultadoNulo.Valores["TRd"] = double.PositiveInfinity;
                resultadoNulo.Valores["indiceTorcao"] = 0.0;

                resultadoNulo.Memoria.Add(
                    "Não há momento torsor solicitante.");

                return resultadoNulo;
            }

            switch (s.Tipo)
            {
                case TipoSecao.TubularCircular:
                    return TorcaoTuboCircular(s, m, p);

                case TipoSecao.TubularRetangular:
                case TipoSecao.Caixao:
                    return TorcaoSecaoFechadaParedesFinas(s, m, p);

                default:
                    throw new NotSupportedException(
                        $"A seção {s.Tipo} não é uma seção fechada suportada.");
            }
        }


        private ResultadoResistencia TorcaoSecaoFechadaParedesFinas(
            SecaoMetalica s,
            MaterialAco m,
            ParametrosTorcao p)
        {
            double alturaExterna =
                (s.H > 0.0 ? s.H : s.D);

            double larguraExterna =
                s.Bf;

            double espessuraAlma =
                s.Tw;

            double espessuraMesa =
                (s.Tf > 0.0 ? s.Tf : s.Tw);

            if (alturaExterna <= 0.0)
            {
                throw new InvalidOperationException(
                    "A altura da seção fechada deve ser maior que zero.");
            }

            if (larguraExterna <= 0.0)
            {
                throw new InvalidOperationException(
                    "A largura da seção fechada deve ser maior que zero.");
            }

            if (espessuraAlma <= 0.0)
            {
                throw new InvalidOperationException(
                    "A espessura das almas deve ser maior que zero.");
            }

            if (espessuraMesa <= 0.0)
            {
                throw new InvalidOperationException(
                    "A espessura das mesas deve ser maior que zero.");
            }

            /*
             * Dimensões da linha média das paredes.
             */
            double alturaMedia =
                alturaExterna - espessuraMesa;

            double larguraMedia =
                larguraExterna - espessuraAlma;

            if (alturaMedia <= 0.0 ||
                larguraMedia <= 0.0)
            {
                throw new InvalidOperationException(
                    "As dimensões da linha média da seção fechada são inválidas.");
            }

            double areaMedia =
                alturaMedia * larguraMedia;

            /*
             * Fluxo de cisalhamento constante em seção unicelular fechada:
             *
             * q = T / (2 Am)
             */
            double fluxoCisalhamento =
                Math.Abs(p.TSd) /
                (2.0 * areaMedia);

            double tauAlmaSd =
                fluxoCisalhamento /
                espessuraAlma;

            double tauMesaSd =
                fluxoCisalhamento /
                espessuraMesa;

            double tauSd =
                Math.Max(
                    tauAlmaSd,
                    tauMesaSd);

            double espessuraCritica =
                Math.Min(
                    espessuraAlma,
                    espessuraMesa);

            double tauRd =
                m.Fy /
                (Math.Sqrt(3.0) * m.GammaA1);

            double indice =
                tauSd / tauRd;

            /*
             * Resistência governada pela parede de menor espessura.
             */
            double tRd =
                2.0 *
                areaMedia *
                espessuraCritica *
                tauRd;

            ResultadoResistencia resultado =
                Novo(
                    tRd,
                    EstadoLimite.Torcao);

            resultado.Valores["TSd"] =
                Math.Abs(p.TSd);

            resultado.Valores["TRd"] =
                tRd;

            resultado.Valores["alturaExterna"] =
                alturaExterna;

            resultado.Valores["larguraExterna"] =
                larguraExterna;

            resultado.Valores["alturaMedia"] =
                alturaMedia;

            resultado.Valores["larguraMedia"] =
                larguraMedia;

            resultado.Valores["areaMedia"] =
                areaMedia;

            resultado.Valores["espessuraAlma"] =
                espessuraAlma;

            resultado.Valores["espessuraMesa"] =
                espessuraMesa;

            resultado.Valores["fluxoCisalhamento"] =
                fluxoCisalhamento;

            resultado.Valores["tauAlmaSd"] =
                tauAlmaSd;

            resultado.Valores["tauMesaSd"] =
                tauMesaSd;

            resultado.Valores["tauSd"] =
                tauSd;

            resultado.Valores["tauRd"] =
                tauRd;

            resultado.Valores["indiceTorcao"] =
                indice;

            resultado.Memoria.Add(
                "Seção fechada unicelular de parede fina.");

            resultado.Memoria.Add(
                "Fluxo de cisalhamento: q = T/(2Am).");

            resultado.Memoria.Add(
                "Tensão torsional em cada parede: τ = q/t.");

            resultado.Memoria.Add(
                "Resistência ao cisalhamento: τRd = fy/(√3·γa1).");

            if (Math.Abs(espessuraAlma - espessuraMesa) > 1.0e-9)
            {
                resultado.Memoria.Add(
                    "A resistência à torção foi governada pela parede de menor espessura.");
            }

            if (indice > 1.0)
            {
                resultado.Avisos.Add(
                    $"Torção não atendida: índice de utilização = {indice:F3}.");
            }

            return resultado;
        }


        private ResultadoResistencia TorcaoTuboCircular(
            SecaoMetalica s,
            MaterialAco m,
            ParametrosTorcao p)
        {
            double diametro =
                (s.Diametro > 0.0 ? s.Diametro : s.D);

            double espessura =
                (s.EspessuraTubo > 0.0 ? s.EspessuraTubo : s.Tw);

            if (diametro <= 0.0)
            {
                throw new InvalidOperationException(
                    "O diâmetro do tubo circular deve ser maior que zero.");
            }

            if (espessura <= 0.0)
            {
                throw new InvalidOperationException(
                    "A espessura do tubo circular deve ser maior que zero.");
            }

            if (2.0 * espessura >= diametro)
            {
                throw new InvalidOperationException(
                    "A espessura do tubo circular é incompatível com o diâmetro.");
            }

            double diametroMedio =
                diametro - espessura;

            double raioMedio =
                diametroMedio / 2.0;

            /*
             * Área delimitada pela linha média da parede.
             */
            double areaMedia =
                Math.PI * raioMedio * raioMedio;

            /*
             * Para seção fechada de parede fina:
             *
             * q = T / (2 Am)
             * tau = q / t
             *
             * Logo:
             * tau = T / (2 Am t)
             */
            double tauSd =
                Math.Abs(p.TSd) /
                (2.0 * areaMedia * espessura);

            double tauRd =
                m.Fy /
                (Math.Sqrt(3.0) * m.GammaA1);

            double indice =
                tauSd / tauRd;

            /*
             * Momento torsor resistente correspondente a tauRd.
             */
            double tRd =
                2.0 *
                areaMedia *
                espessura *
                tauRd;

            ResultadoResistencia resultado =
                Novo(
                    tRd,
                    EstadoLimite.Torcao);

            resultado.Valores["TSd"] =
                Math.Abs(p.TSd);

            resultado.Valores["TRd"] =
                tRd;

            resultado.Valores["diametroExterno"] =
                diametro;

            resultado.Valores["diametroMedio"] =
                diametroMedio;

            resultado.Valores["espessura"] =
                espessura;

            resultado.Valores["areaMedia"] =
                areaMedia;

            resultado.Valores["tauSd"] =
                tauSd;

            resultado.Valores["tauRd"] =
                tauRd;

            resultado.Valores["indiceTorcao"] =
                indice;

            resultado.Memoria.Add(
                "Tubo circular de parede fina: q = T/(2Am).");

            resultado.Memoria.Add(
                "Tensão de cisalhamento torsional: τSd = T/(2Am·t).");

            resultado.Memoria.Add(
                "Resistência ao cisalhamento: τRd = fy/(√3·γa1).");

            if (indice > 1.0)
            {
                resultado.Avisos.Add(
                    $"Torção não atendida: índice de utilização = {indice:F3}.");
            }

            return resultado;
        }



        private static bool EhSecaoFechada(TipoSecao tipo)
        {
            switch (tipo)
            {
                case TipoSecao.TubularCircular:
                case TipoSecao.TubularRetangular:
                case TipoSecao.Caixao:
                    return true;

                default:
                    return false;
            }
        }

        private ResultadoResistencia TorcaoSecaoAberta(
    SecaoMetalica s,
    MaterialAco m,
    ParametrosTorcao p)
        {
            const double tolerancia = 1.0e-12;

            if (s == null)
                throw new ArgumentNullException(nameof(s));

            if (m == null)
                throw new ArgumentNullException(nameof(m));

            if (p == null)
                throw new ArgumentNullException(nameof(p));

            if (s.J <= tolerancia)
            {
                throw new InvalidOperationException(
                    $"A constante de torção J não foi informada para a seção {s.Tipo}.");
            }

            double tMax = ObterEspessuraMaxima(s);

            if (tMax <= tolerancia)
            {
                throw new InvalidOperationException(
                    $"Não foi possível determinar a espessura máxima da seção {s.Tipo}.");
            }

            /*
             * Determinação da parcela de torção uniforme de Saint-Venant.
             *
             * Quando a análise fornece TSaintVenantSd, utiliza-se diretamente
             * essa parcela.
             *
             * Quando o empenamento é livre e a parcela não foi separada,
             * admite-se que todo o torsor TSd seja resistido por Saint-Venant.
             */
            double tSaintVenant;

            if (Math.Abs(p.TSaintVenantSd) > tolerancia)
            {
                tSaintVenant = p.TSaintVenantSd;
            }
            else if (p.EmpenamentoLivre)
            {
                tSaintVenant = p.TSd;
            }
            else
            {
                throw new InvalidOperationException(
                    "Perfil aberto com empenamento restringido: informe a parcela " +
                    "TSaintVenantSd obtida pela análise de torção não uniforme.");
            }

            double tauSaintVenant =
                Math.Abs(tSaintVenant) *
                tMax /
                s.J;

            /*
             * Tensões decorrentes da restrição ao empenamento.
             *
             * OmegaMax e Cw somente são necessários quando existe bimomento
             * diferente de zero.
             */
            double omegaMax = 0.0;
            double sigmaEmpenamento = 0.0;
            bool possuiBimomento =
                Math.Abs(p.BSd) > tolerancia;

            if (!p.EmpenamentoLivre)
            {
                if (!p.AnaliseEmpenamentoRealizada)
                {
                    throw new InvalidOperationException(
                        "O empenamento está restringido, mas a análise de torção " +
                        "não uniforme não foi realizada.");
                }

                if (possuiBimomento)
                {
                    if (s.Cw <= tolerancia)
                    {
                        throw new InvalidOperationException(
                            $"A constante de empenamento Cw não foi informada " +
                            $"para a seção aberta {s.Tipo}.");
                    }

                    omegaMax =
                        p.OmegaMax > tolerancia
                            ? p.OmegaMax
                            : s.OmegaMax;

                    if (omegaMax <= tolerancia)
                    {
                        throw new InvalidOperationException(
                            $"A coordenada setorial máxima não foi informada " +
                            $"para a seção aberta {s.Tipo}.");
                    }

                    sigmaEmpenamento =
                        Math.Abs(p.BSd) *
                        omegaMax /
                        s.Cw;
                }
            }

            /*
             * Combinação conservadora em módulo.
             *
             * Em uma análise mais completa, devem ser avaliados os sinais das
             * tensões em cada ponto da seção transversal.
             */
            double sigmaNormal =
                Math.Abs(p.SigmaNormalSd);

            double tauCortante =
                Math.Abs(p.TauCortanteSd);

            double tauEmpenamento =
                Math.Abs(p.TauEmpenamentoSd);

            double sigmaTotal =
                sigmaNormal +
                sigmaEmpenamento;

            double tauTotal =
                tauCortante +
                tauSaintVenant +
                tauEmpenamento;

            double sigmaEquivalente =
                Math.Sqrt(
                    sigmaTotal * sigmaTotal +
                    3.0 * tauTotal * tauTotal);

            if (m.Fy <= tolerancia)
            {
                throw new InvalidOperationException(
                    "A resistência ao escoamento Fy do aço não foi informada.");
            }

            if (m.GammaA1 <= tolerancia)
            {
                throw new InvalidOperationException(
                    "O coeficiente de ponderação GammaA1 deve ser maior que zero.");
            }

            double sigmaRd =
                m.Fy /
                m.GammaA1;

            double indice =
                sigmaEquivalente /
                sigmaRd;

            ResultadoResistencia resultado =
                Novo(
                    sigmaRd,
                    EstadoLimite.Torcao);

            resultado.Valores["TSd"] =
                p.TSd;

            resultado.Valores["TSaintVenantSd"] =
                tSaintVenant;

            resultado.Valores["BSd"] =
                p.BSd;

            resultado.Valores["J"] =
                s.J;

            resultado.Valores["Cw"] =
                s.Cw;

            resultado.Valores["omegaMax"] =
                omegaMax;

            resultado.Valores["tMax"] =
                tMax;

            resultado.Valores["sigmaNormalSd"] =
                sigmaNormal;

            resultado.Valores["tauCortanteSd"] =
                tauCortante;

            resultado.Valores["tauSaintVenant"] =
                tauSaintVenant;

            resultado.Valores["tauEmpenamento"] =
                tauEmpenamento;

            resultado.Valores["sigmaEmpenamento"] =
                sigmaEmpenamento;

            resultado.Valores["sigmaTotal"] =
                sigmaTotal;

            resultado.Valores["tauTotal"] =
                tauTotal;

            resultado.Valores["sigmaEquivalente"] =
                sigmaEquivalente;

            resultado.Valores["sigmaRd"] =
                sigmaRd;

            resultado.Valores["indiceTorcao"] =
                indice;

            resultado.Memoria.Add(
                "Perfil aberto: verificação por tensões considerando " +
                "torção uniforme de Saint-Venant e, quando aplicável, " +
                "torção não uniforme com restrição ao empenamento.");

            resultado.Memoria.Add(
                "τt,Sd = |Tsv,Sd| · tmax / J.");

            if (!p.EmpenamentoLivre)
            {
                resultado.Memoria.Add(
                    "O empenamento foi considerado restringido.");

                if (possuiBimomento)
                {
                    resultado.Memoria.Add(
                        "σw,Sd = |BSd| · |ω|max / Cw.");

                    resultado.Memoria.Add(
                        "Foi considerada tensão normal adicional decorrente " +
                        "do bimomento de empenamento.");
                }
                else
                {
                    resultado.Memoria.Add(
                        "A análise de torção não uniforme foi realizada, porém " +
                        "o bimomento BSd é nulo; portanto, σw,Sd = 0.");
                }

                if (tauEmpenamento > tolerancia)
                {
                    resultado.Memoria.Add(
                        "Foi considerada tensão de cisalhamento adicional " +
                        "decorrente do empenamento.");
                }
            }
            else
            {
                resultado.Memoria.Add(
                    "Empenamento considerado livre; não foram adicionadas " +
                    "tensões normais de empenamento.");
            }

            resultado.Memoria.Add(
                "σtotal,Sd = |σnormal,Sd| + |σw,Sd|.");

            resultado.Memoria.Add(
                "τtotal,Sd = |τcortante,Sd| + |τt,Sd| + |τw,Sd|.");

            resultado.Memoria.Add(
                "σeq,Sd = √(σtotal,Sd² + 3·τtotal,Sd²).");

            resultado.Memoria.Add(
                "σRd = fy / γa1.");

            resultado.Memoria.Add(
                "Índice de utilização = σeq,Sd / σRd.");

            if (indice > 1.0)
            {
                resultado.Avisos.Add(
                    $"Verificação à torção não atendida: índice = {indice:F3}.");
            }

            VerificarRotacaoTorcional(
                p,
                resultado);

            return resultado;
        }

        private static double ObterEspessuraMaxima(SecaoMetalica s)
        {            
            if (s.EspessuraMaxima > 0.0)
                return s.EspessuraMaxima;

            return Math.Max(Math.Max(s.Tw, s.Tf), s.EspessuraTubo);
        }

        private static void VerificarRotacaoTorcional(
    ParametrosTorcao p,
    ResultadoResistencia resultado)
        {
            if (p.RotacaoMaxRad <= 0.0 ||
                p.RotacaoLimiteRad <= 0.0)
            {
                resultado.Avisos.Add(
                    "Rotação torsional não verificada: informe a rotação " +
                    "calculada e o limite de serviço adotado.");
                return;
            }

            double indiceRotacao =
                p.RotacaoMaxRad / p.RotacaoLimiteRad;

            resultado.Valores["rotacaoMaxRad"] =
                p.RotacaoMaxRad;

            resultado.Valores["rotacaoLimiteRad"] =
                p.RotacaoLimiteRad;

            resultado.Valores["indiceRotacao"] =
                indiceRotacao;

            if (indiceRotacao > 1.0)
            {
                resultado.Avisos.Add(
                    $"Rotação torsional acima do limite: índice = {indiceRotacao:F3}.");
            }
        }
        private static void Validar(SecaoMetalica s, MaterialAco m)
        {
            if (s == null || m == null) throw new ArgumentNullException();
            if (!FinitoPositivo(s.Ag) || !FinitoPositivo(s.Ix) || !FinitoPositivo(s.Iy) ||
                !FinitoPositivo(m.Fy) || !FinitoPositivo(m.E) || !FinitoPositivo(m.G) ||
                !FinitoPositivo(m.GammaA1) || !FinitoPositivo(m.GammaA2))
                throw new ArgumentException("Propriedades básicas devem ser positivas e finitas.");
        }

        private static bool FinitoPositivo(double v) { return v > 0 && !double.IsNaN(v) && !double.IsInfinity(v); }

        private static double MenorRaizPositivaCubicBuckling(double nx, double ny, double nz, double x0, double y0, double r02)
        {
            // Varredura/bisseção robusta da equação de 5.3.5.3 no intervalo físico (0,min Euler].
            Func<double, double> f = n => (n - nx) * (n - ny) * (n - nz) - ny * (n - nx) * x0 * x0 / r02 - nx * (n - ny) * y0 * y0 / r02;
            double max = Math.Min(nx, Math.Min(ny, nz)), lastX = 0, lastF = f(0);
            for (int i = 1; i <= 2000; i++)
            {
                double x = max * i / 2000.0, fx = f(x);
                if (fx == 0 || fx * lastF < 0)
                {
                    double lo = lastX, hi = x;
                    for (int k = 0; k < 80; k++) { double mid = (lo + hi) / 2; if (f(lo) * f(mid) <= 0) hi = mid; else lo = mid; }
                    return (lo + hi) / 2;
                }
                lastX = x; lastF = fx;
            }
            throw new InvalidOperationException("Não foi possível determinar a raiz positiva de flambagem.");
        }
    }
}
