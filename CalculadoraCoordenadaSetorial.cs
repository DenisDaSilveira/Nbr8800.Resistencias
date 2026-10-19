using System;

namespace Nbr8800.Resistencias
{
    public static class CalculadoraCoordenadaSetorial
    {
        private const double Tolerancia = 1.0e-12;

        /// <summary>
        /// Preenche e retorna a coordenada setorial máxima da seção.
        ///
        /// Para seções abertas sem fórmula implementada, preserva um valor
        /// anteriormente informado em SecaoMetalica.OmegaMax.
        ///
        /// Unidades:
        /// se as dimensões estiverem em cm, OmegaMax será fornecido em cm²;
        /// se as dimensões estiverem em mm, OmegaMax será fornecido em mm².
        /// </summary>
        public static double CalcularOmegaMax(
            SecaoMetalica secao,
            bool exigirParaSecaoAberta = true)
        {
            if (secao == null)
                throw new ArgumentNullException(nameof(secao));

            double omegaMax;

            switch (secao.Tipo)
            {
                /*
                 * Perfil I/H duplamente simétrico, idealizado como
                 * seção aberta de paredes finas.
                 */
                case TipoSecao.IouHDuplaSimetria:
                    omegaMax =
                        CalcularPerfilIDuplaSimetria(secao);
                    break;

                /*
                 * Para estes tipos, as propriedades disponíveis na classe
                 * não são suficientes para determinar corretamente ωmáx.
                 *
                 * Se OmegaMax já tiver sido calculado por catálogo, rotina
                 * geométrica ou solver de seção, o valor é preservado.
                 */
                case TipoSecao.IouHMonossimetrica:
                case TipoSecao.U:
                case TipoSecao.T:
                case TipoSecao.DuasCantoneirasEmT:
                case TipoSecao.Cantoneira:
                case TipoSecao.GenericaDuplaSimetria:
                case TipoSecao.GenericaMonossimetrica:
                case TipoSecao.GenericaAssimetrica:

                    omegaMax =
                        ObterOmegaInformadoOuFalhar(
                            secao,
                            exigirParaSecaoAberta);
                    break;

                /*
                 * Nas rotinas atuais para seções fechadas, a resistência
                 * torsional é tratada por fluxo de cisalhamento/Bredt.
                 * OmegaMax não é utilizado.
                 */
                case TipoSecao.TubularRetangular:
                case TipoSecao.Caixao:
                case TipoSecao.TubularCircular:
                case TipoSecao.SolidaRetangular:
                case TipoSecao.SolidaCircular:
                    omegaMax = 0.0;
                    break;

                default:
                    throw new ArgumentOutOfRangeException(
                        nameof(secao.Tipo),
                        secao.Tipo,
                        "Tipo de seção metálica desconhecido.");
            }

            secao.OmegaMax = omegaMax;

            return omegaMax;
        }

        private static double CalcularPerfilIDuplaSimetria(
            SecaoMetalica secao)
        {
            ValidarPositivo(
                secao.H,
                nameof(secao.H),
                secao.Tipo);

            ValidarPositivo(
                secao.Bf,
                nameof(secao.Bf),
                secao.Tipo);

            ValidarPositivo(
                secao.Tf,
                nameof(secao.Tf),
                secao.Tipo);

            /*
             * Distância entre as linhas médias das mesas:
             *
             * h0 = H - Tf
             *
             * Como existem duas mesas de mesma espessura:
             * H - Tf/2 - Tf/2 = H - Tf.
             */
            double h0 =
                secao.H - secao.Tf;

            if (h0 <= Tolerancia)
            {
                throw new InvalidOperationException(
                    $"Não foi possível calcular OmegaMax para {secao.Tipo}: " +
                    $"a distância entre as linhas médias das mesas é inválida. " +
                    $"H={secao.H}; Tf={secao.Tf}; h0={h0}.");
            }

            /*
             * Perfil I duplamente simétrico:
             *
             * |ω|max = h0 · bf / 4
             *
             * Hipóteses:
             * - seção idealizada por linhas médias;
             * - mesas de mesma largura e espessura;
             * - alma centralizada;
             * - origem e normalização setorial compatíveis com Cw.
             */
            double omegaMax =
                h0 * secao.Bf / 4.0;

            if (!EhNumeroPositivoFinito(omegaMax))
            {
                throw new InvalidOperationException(
                    $"OmegaMax calculado inválido para {secao.Tipo}: " +
                    $"{omegaMax}.");
            }

            return omegaMax;
        }

        private static double ObterOmegaInformadoOuFalhar(
            SecaoMetalica secao,
            bool exigir)
        {
            /*
             * Permite que o valor venha:
             * - do catálogo do perfil;
             * - de uma rotina geométrica;
             * - de um solver de propriedades da seção;
             * - de dados fornecidos pelo usuário.
             */
            if (EhNumeroPositivoFinito(secao.OmegaMax))
                return secao.OmegaMax;

            if (!exigir)
                return 0.0;

            throw new InvalidOperationException(
                $"Não é possível calcular automaticamente OmegaMax para a " +
                $"seção {secao.Tipo} usando apenas H, Bf, Tf e Tw. " +
                $"Informe SecaoMetalica.OmegaMax ou execute uma análise " +
                $"setorial do contorno médio da seção.");
        }

        private static void ValidarPositivo(
            double valor,
            string nome,
            TipoSecao tipo)
        {
            if (!EhNumeroPositivoFinito(valor))
            {
                throw new InvalidOperationException(
                    $"{nome} deve ser positivo e finito para calcular " +
                    $"OmegaMax da seção {tipo}. Valor recebido: {valor}.");
            }
        }

        private static bool EhNumeroPositivoFinito(
            double valor)
        {
            return
                !double.IsNaN(valor) &&
                !double.IsInfinity(valor) &&
                valor > Tolerancia;
        }
    }
}
