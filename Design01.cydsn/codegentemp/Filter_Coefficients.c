#include "Filter.h"
#include "Filter_PVT.h"


/*******************************************************************************
* ChannelA filter coefficients.
* Filter Type is FIR
*******************************************************************************/

/* Renamed array for backward compatibility.
*  Should not be used in new designs.
*/
#define ChannelAFirCoefficients Filter_ChannelAFirCoefficients

/* Number of FIR filter taps are: 31 */

const uint8 CYCODE Filter_ChannelAFirCoefficients[Filter_FIR_A_SIZE] = 
{
 0x7Bu, 0x03u, 0x00u, 0x00u, /* Tap(0), 0.000106215476989746 */

 0xD6u, 0xEFu, 0xFFu, 0x00u, /* Tap(1), -0.000493288040161133 */

 0x1Bu, 0x14u, 0x00u, 0x00u, /* Tap(2), 0.000613570213317871 */

 0xE9u, 0x1Du, 0x00u, 0x00u, /* Tap(3), 0.000912785530090332 */

 0x45u, 0x82u, 0xFFu, 0x00u, /* Tap(4), -0.003836989402771 */

 0xEFu, 0x80u, 0x00u, 0x00u, /* Tap(5), 0.00393474102020264 */

 0x64u, 0x75u, 0x00u, 0x00u, /* Tap(6), 0.00358247756958008 */

 0x72u, 0x14u, 0xFEu, 0x00u, /* Tap(7), -0.0150010585784912 */

 0xB7u, 0xE4u, 0x01u, 0x00u, /* Tap(8), 0.0147923231124878 */

 0x10u, 0x42u, 0x01u, 0x00u, /* Tap(9), 0.00982856750488281 */

 0xE7u, 0x4Eu, 0xFAu, 0x00u, /* Tap(10), -0.0444670915603638 */

 0x0Bu, 0x98u, 0x05u, 0x00u, /* Tap(11), 0.0437024831771851 */

 0x9Au, 0xD1u, 0x03u, 0x00u, /* Tap(12), 0.0298340320587158 */

 0x23u, 0xFCu, 0xEDu, 0x00u, /* Tap(13), -0.140742897987366 */

 0x7Au, 0xBEu, 0x0Fu, 0x00u, /* Tap(14), 0.123000383377075 */

 0x62u, 0xB3u, 0x38u, 0x00u, /* Tap(15), 0.442974328994751 */

 0x7Au, 0xBEu, 0x0Fu, 0x00u, /* Tap(16), 0.123000383377075 */

 0x23u, 0xFCu, 0xEDu, 0x00u, /* Tap(17), -0.140742897987366 */

 0x9Au, 0xD1u, 0x03u, 0x00u, /* Tap(18), 0.0298340320587158 */

 0x0Bu, 0x98u, 0x05u, 0x00u, /* Tap(19), 0.0437024831771851 */

 0xE7u, 0x4Eu, 0xFAu, 0x00u, /* Tap(20), -0.0444670915603638 */

 0x10u, 0x42u, 0x01u, 0x00u, /* Tap(21), 0.00982856750488281 */

 0xB7u, 0xE4u, 0x01u, 0x00u, /* Tap(22), 0.0147923231124878 */

 0x72u, 0x14u, 0xFEu, 0x00u, /* Tap(23), -0.0150010585784912 */

 0x64u, 0x75u, 0x00u, 0x00u, /* Tap(24), 0.00358247756958008 */

 0xEFu, 0x80u, 0x00u, 0x00u, /* Tap(25), 0.00393474102020264 */

 0x45u, 0x82u, 0xFFu, 0x00u, /* Tap(26), -0.003836989402771 */

 0xE9u, 0x1Du, 0x00u, 0x00u, /* Tap(27), 0.000912785530090332 */

 0x1Bu, 0x14u, 0x00u, 0x00u, /* Tap(28), 0.000613570213317871 */

 0xD6u, 0xEFu, 0xFFu, 0x00u, /* Tap(29), -0.000493288040161133 */

 0x7Bu, 0x03u, 0x00u, 0x00u, /* Tap(30), 0.000106215476989746 */
};


/*******************************************************************************
* ChannelB filter coefficients.
* Filter Type is FIR
*******************************************************************************/

/* Renamed array for backward compatibility.
*  Should not be used in new designs.
*/
#define ChannelBFirCoefficients Filter_ChannelBFirCoefficients

/* Number of FIR filter taps are: 31 */

const uint8 CYCODE Filter_ChannelBFirCoefficients[Filter_FIR_B_SIZE] = 
{
 0x7Bu, 0x03u, 0x00u, 0x00u, /* Tap(0), 0.000106215476989746 */

 0xD6u, 0xEFu, 0xFFu, 0x00u, /* Tap(1), -0.000493288040161133 */

 0x1Bu, 0x14u, 0x00u, 0x00u, /* Tap(2), 0.000613570213317871 */

 0xE9u, 0x1Du, 0x00u, 0x00u, /* Tap(3), 0.000912785530090332 */

 0x45u, 0x82u, 0xFFu, 0x00u, /* Tap(4), -0.003836989402771 */

 0xEFu, 0x80u, 0x00u, 0x00u, /* Tap(5), 0.00393474102020264 */

 0x64u, 0x75u, 0x00u, 0x00u, /* Tap(6), 0.00358247756958008 */

 0x72u, 0x14u, 0xFEu, 0x00u, /* Tap(7), -0.0150010585784912 */

 0xB7u, 0xE4u, 0x01u, 0x00u, /* Tap(8), 0.0147923231124878 */

 0x10u, 0x42u, 0x01u, 0x00u, /* Tap(9), 0.00982856750488281 */

 0xE7u, 0x4Eu, 0xFAu, 0x00u, /* Tap(10), -0.0444670915603638 */

 0x0Bu, 0x98u, 0x05u, 0x00u, /* Tap(11), 0.0437024831771851 */

 0x9Au, 0xD1u, 0x03u, 0x00u, /* Tap(12), 0.0298340320587158 */

 0x23u, 0xFCu, 0xEDu, 0x00u, /* Tap(13), -0.140742897987366 */

 0x7Au, 0xBEu, 0x0Fu, 0x00u, /* Tap(14), 0.123000383377075 */

 0x62u, 0xB3u, 0x38u, 0x00u, /* Tap(15), 0.442974328994751 */

 0x7Au, 0xBEu, 0x0Fu, 0x00u, /* Tap(16), 0.123000383377075 */

 0x23u, 0xFCu, 0xEDu, 0x00u, /* Tap(17), -0.140742897987366 */

 0x9Au, 0xD1u, 0x03u, 0x00u, /* Tap(18), 0.0298340320587158 */

 0x0Bu, 0x98u, 0x05u, 0x00u, /* Tap(19), 0.0437024831771851 */

 0xE7u, 0x4Eu, 0xFAu, 0x00u, /* Tap(20), -0.0444670915603638 */

 0x10u, 0x42u, 0x01u, 0x00u, /* Tap(21), 0.00982856750488281 */

 0xB7u, 0xE4u, 0x01u, 0x00u, /* Tap(22), 0.0147923231124878 */

 0x72u, 0x14u, 0xFEu, 0x00u, /* Tap(23), -0.0150010585784912 */

 0x64u, 0x75u, 0x00u, 0x00u, /* Tap(24), 0.00358247756958008 */

 0xEFu, 0x80u, 0x00u, 0x00u, /* Tap(25), 0.00393474102020264 */

 0x45u, 0x82u, 0xFFu, 0x00u, /* Tap(26), -0.003836989402771 */

 0xE9u, 0x1Du, 0x00u, 0x00u, /* Tap(27), 0.000912785530090332 */

 0x1Bu, 0x14u, 0x00u, 0x00u, /* Tap(28), 0.000613570213317871 */

 0xD6u, 0xEFu, 0xFFu, 0x00u, /* Tap(29), -0.000493288040161133 */

 0x7Bu, 0x03u, 0x00u, 0x00u, /* Tap(30), 0.000106215476989746 */
};

