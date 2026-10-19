using System;
using System.Collections.Generic;

namespace Nbr8800.Resistencias
{
    /// <summary>
    /// Dados de uma barra prismática para análise linear de torção não uniforme.
    /// Unidades obrigatórias: N, mm e MPa (N/mm²).
    /// </summary>
    public sealed class DadosAnaliseTorcaoVlasov
    {
        public double Comprimento { get; set; }
        public double E { get; set; }
        public double G { get; set; }
        public double J { get; set; }
        public double Cw { get; set; }
        public int NumeroElementos { get; set; } = 20;
        /// <summary>Momento torsor distribuído por comprimento (N.mm/mm = N).</summary>
        public double MomentoTorsorDistribuido { get; set; }
        public double MomentoTorsorInicial { get; set; }
        public double MomentoTorsorFinal { get; set; }
        public CondicoesTorsionaisViga Condicoes { get; set; } = new CondicoesTorsionaisViga();
    }

    public sealed class PontoResultadoTorcaoVlasov
    {
        public double X { get; set; }
        public double Rotacao { get; set; }
        public double DerivadaRotacao { get; set; }
        public double TorsorSaintVenant { get; set; }
        public double TorsorEmpenamento { get; set; }
        public double Bimomento { get; set; }
        public double TorsorTotal => TorsorSaintVenant + TorsorEmpenamento;
    }

    public sealed class ResultadoAnaliseTorcaoVlasov
    {
        public List<PontoResultadoTorcaoVlasov> Pontos { get; } =
            new List<PontoResultadoTorcaoVlasov>();
        public double RotacaoMaxima { get; internal set; }
        public double TorsorSaintVenantMaximo { get; internal set; }
        public double TorsorEmpenamentoMaximo { get; internal set; }
        public double BimomentoMaximo { get; internal set; }
    }

    /// <summary>
    /// Elemento finito de Vlasov com quatro graus de liberdade
    /// {theta1, theta'1, theta2, theta'2}.
    /// </summary>
    public sealed class AnaliseTorcaoVlasov
    {
        private const double Tolerancia = 1.0e-12;

        public ResultadoAnaliseTorcaoVlasov Calcular(DadosAnaliseTorcaoVlasov d)
        {
            Validar(d);
            int ne = Math.Max(1, d.NumeroElementos);
            int nn = ne + 1;
            int ngl = 2 * nn;
            double le = d.Comprimento / ne;
            var k = new double[ngl, ngl];
            var f = new double[ngl];

            for (int e = 0; e < ne; e++)
            {
                double[,] ke = MatrizElemento(d.E * d.Cw, d.G * d.J,
                    d.Condicoes?.KThetaDistribuido ?? 0.0, le);
                double[] fe = VetorCargaUniforme(d.MomentoTorsorDistribuido, le);
                int[] mapa = { 2 * e, 2 * e + 1, 2 * e + 2, 2 * e + 3 };
                for (int i = 0; i < 4; i++)
                {
                    f[mapa[i]] += fe[i];
                    for (int j = 0; j < 4; j++) k[mapa[i], mapa[j]] += ke[i, j];
                }
            }

            f[0] += d.MomentoTorsorInicial;
            f[2 * (nn - 1)] += d.MomentoTorsorFinal;
            AplicarExtremidade(k, d.Condicoes?.Inicial, 0, 1);
            AplicarExtremidade(k, d.Condicoes?.Final, ngl - 2, ngl - 1);

            var restritos = new bool[ngl];
            MarcarRestricoes(restritos, d.Condicoes?.Inicial, 0, 1);
            MarcarRestricoes(restritos, d.Condicoes?.Final, ngl - 2, ngl - 1);
            double[] u = ResolverReduzido(k, f, restritos);
            return PosProcessar(d, u, ne, le);
        }

        public static double[,] MatrizElemento(double eCw, double gJ,
            double kThetaDistribuido, double l)
        {
            if (l <= 0.0) throw new ArgumentOutOfRangeException(nameof(l));
            var r = new double[4, 4];
            double l2 = l * l;
            double l3 = l2 * l;
            double[,] kw =
            {
                {12, 6*l, -12, 6*l},
                {6*l, 4*l2, -6*l, 2*l2},
                {-12, -6*l, 12, -6*l},
                {6*l, 2*l2, -6*l, 4*l2}
            };
            double[,] kt =
            {
                {36, 3*l, -36, 3*l},
                {3*l, 4*l2, -3*l, -l2},
                {-36, -3*l, 36, -3*l},
                {3*l, -l2, -3*l, 4*l2}
            };
            double[,] kd =
            {
                {156, 22*l, 54, -13*l},
                {22*l, 4*l2, 13*l, -3*l2},
                {54, 13*l, 156, -22*l},
                {-13*l, -3*l2, -22*l, 4*l2}
            };
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    r[i, j] = eCw / l3 * kw[i, j]
                        + gJ / (30.0 * l) * kt[i, j]
                        + kThetaDistribuido * l / 420.0 * kd[i, j];
            return r;
        }

        public static double[] VetorCargaUniforme(double mt, double l) =>
            new[] { mt * l / 2.0, mt * l * l / 12.0,
                    mt * l / 2.0, -mt * l * l / 12.0 };

        private static void AplicarExtremidade(double[,] k,
            RestricaoExtremidadeTorcional r, int glTheta, int glOmega)
        {
            if (r == null) return;
            if (r.Rotacao == CondicaoRotacaoTorcional.Elastico)
            {
                if (r.KTheta <= 0.0) throw new InvalidOperationException(
                    "KTheta deve ser positivo para uma restrição rotacional elástica.");
                k[glTheta, glTheta] += r.KTheta;
            }
            if (r.Empenamento == CondicaoEmpenamentoExtremidade.Elastico)
            {
                if (r.KOmega <= 0.0) throw new InvalidOperationException(
                    "KOmega deve ser positivo para uma restrição elástica ao empenamento.");
                k[glOmega, glOmega] += r.KOmega;
            }
        }

        private static void MarcarRestricoes(bool[] restritos,
            RestricaoExtremidadeTorcional r, int glTheta, int glOmega)
        {
            if (r == null) return;
            restritos[glTheta] = r.Rotacao == CondicaoRotacaoTorcional.Impedida;
            restritos[glOmega] = r.Empenamento == CondicaoEmpenamentoExtremidade.Impedido;
        }

        private static double[] ResolverReduzido(double[,] k, double[] f, bool[] restritos)
        {
            var livres = new List<int>();
            for (int i = 0; i < restritos.Length; i++) if (!restritos[i]) livres.Add(i);
            if (livres.Count == 0) return new double[restritos.Length];
            var a = new double[livres.Count, livres.Count];
            var b = new double[livres.Count];
            for (int i = 0; i < livres.Count; i++)
            {
                b[i] = f[livres[i]];
                for (int j = 0; j < livres.Count; j++) a[i, j] = k[livres[i], livres[j]];
            }
            double[] x = ResolverGauss(a, b);
            var u = new double[restritos.Length];
            for (int i = 0; i < livres.Count; i++) u[livres[i]] = x[i];
            return u;
        }

        private static double[] ResolverGauss(double[,] a, double[] b)
        {
            int n = b.Length;
            for (int p = 0; p < n; p++)
            {
                int imax = p;
                for (int i = p + 1; i < n; i++)
                    if (Math.Abs(a[i, p]) > Math.Abs(a[imax, p])) imax = i;
                if (Math.Abs(a[imax, p]) <= Tolerancia)
                    throw new InvalidOperationException(
                        "Sistema torsional singular: faltam restrições à rotação torsional.");
                if (imax != p)
                {
                    for (int j = p; j < n; j++)
                    { double t = a[p, j]; a[p, j] = a[imax, j]; a[imax, j] = t; }
                    double tb = b[p]; b[p] = b[imax]; b[imax] = tb;
                }
                for (int i = p + 1; i < n; i++)
                {
                    double m = a[i, p] / a[p, p];
                    b[i] -= m * b[p];
                    for (int j = p; j < n; j++) a[i, j] -= m * a[p, j];
                }
            }
            var x = new double[n];
            for (int i = n - 1; i >= 0; i--)
            {
                double s = b[i];
                for (int j = i + 1; j < n; j++) s -= a[i, j] * x[j];
                x[i] = s / a[i, i];
            }
            return x;
        }

        private static ResultadoAnaliseTorcaoVlasov PosProcessar(
            DadosAnaliseTorcaoVlasov d, double[] u, int ne, double le)
        {
            var r = new ResultadoAnaliseTorcaoVlasov();
            for (int e = 0; e < ne; e++)
            {
                double[] ue = { u[2 * e], u[2 * e + 1], u[2 * e + 2], u[2 * e + 3] };
                int inicio = e == 0 ? 0 : 1;
                for (int ip = inicio; ip <= 4; ip++)
                {
                    double xi = ip / 4.0;
                    double[] n = Funcoes(xi, le, 0);
                    double[] d1 = Funcoes(xi, le, 1);
                    double[] d2 = Funcoes(xi, le, 2);
                    double[] d3 = Funcoes(xi, le, 3);
                    double theta = Produto(n, ue);
                    double psi = Produto(d1, ue);
                    double tsv = d.G * d.J * psi;
                    double bimomento = -d.E * d.Cw * Produto(d2, ue);
                    double tw = -d.E * d.Cw * Produto(d3, ue);
                    r.Pontos.Add(new PontoResultadoTorcaoVlasov {
                        X = e * le + xi * le, Rotacao = theta,
                        DerivadaRotacao = psi, TorsorSaintVenant = tsv,
                        TorsorEmpenamento = tw, Bimomento = bimomento });
                    r.RotacaoMaxima = Math.Max(r.RotacaoMaxima, Math.Abs(theta));
                    r.TorsorSaintVenantMaximo = Math.Max(r.TorsorSaintVenantMaximo, Math.Abs(tsv));
                    r.TorsorEmpenamentoMaximo = Math.Max(r.TorsorEmpenamentoMaximo, Math.Abs(tw));
                    r.BimomentoMaximo = Math.Max(r.BimomentoMaximo, Math.Abs(bimomento));
                }
            }
            return r;
        }

        private static double[] Funcoes(double x, double l, int ordem)
        {
            if (ordem == 0) return new[] { 1-3*x*x+2*x*x*x, l*(x-2*x*x+x*x*x),
                                           3*x*x-2*x*x*x, l*(-x*x+x*x*x) };
            if (ordem == 1) return new[] { (-6*x+6*x*x)/l, 1-4*x+3*x*x,
                                           (6*x-6*x*x)/l, -2*x+3*x*x };
            if (ordem == 2) return new[] { (-6+12*x)/(l*l), (-4+6*x)/l,
                                           (6-12*x)/(l*l), (-2+6*x)/l };
            return new[] { 12/(l*l*l), 6/(l*l), -12/(l*l*l), 6/(l*l) };
        }

        private static double Produto(double[] a, double[] b)
        {
            double r = 0.0;
            for (int i = 0; i < a.Length; i++) r += a[i] * b[i];
            return r;
        }

        private static void Validar(DadosAnaliseTorcaoVlasov d)
        {
            if (d == null) throw new ArgumentNullException(nameof(d));
            if (d.Comprimento <= 0 || d.E <= 0 || d.G <= 0 || d.J <= 0 || d.Cw <= 0)
                throw new ArgumentException("L, E, G, J e Cw devem ser positivos.");
            if (d.Condicoes == null) d.Condicoes = new CondicoesTorsionaisViga();
            if (d.Condicoes.KThetaDistribuido < 0)
                throw new ArgumentException("KThetaDistribuido não pode ser negativo.");
        }
    }
}
