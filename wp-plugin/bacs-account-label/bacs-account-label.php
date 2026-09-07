<?php
/**
 * Plugin Name:       BACS Account Label (fa)
 * Plugin URI:        https://github.com/pedramhimself/Claude
 * Description:       تغییر برچسب «شماره کارت» به «شماره حساب» در باکس «مشخصات بانکی ما» (درگاه انتقال بانکی مستقیم ووکامرس) در صفحه ثبت سفارش و ایمیل‌های سفارش.
 * Version:           1.0.0
 * Requires at least: 5.6
 * Requires PHP:      7.2
 * Author:            Danesh Pazhouhan Faraz Andisheh
 * Text Domain:       bacs-account-label
 * License:           GPL-2.0-or-later
 */

defined( 'ABSPATH' ) || exit;

/**
 * برچسب‌های دلخواه برای فیلدهای حساب بانکی.
 *
 * کلیدهای معتبر: bank_name, account_number, sort_code, iban, bic
 * برای تغییر هر برچسب، مقدار روبه‌روی آن را ویرایش کنید.
 * برای دست‌نخورده ماندن یک فیلد، خط مربوط به آن را حذف کنید.
 */
function bacs_account_label_map() {
	return apply_filters(
		'bacs_account_label_map',
		array(
			'account_number' => 'شماره حساب',
		)
	);
}

/**
 * برچسب‌ها را روی خروجی درگاه BACS اعمال می‌کند.
 *
 * این فیلتر هم در صفحه «سفارش دریافت شد» و هم در ایمیل سفارش اجرا می‌شود،
 * چون هر دو از WC_Gateway_BACS::bank_details() می‌آیند.
 *
 * @param array $fields   فیلدهای حساب بانکی.
 * @param int   $order_id شناسه سفارش.
 * @return array
 */
function bacs_account_label_filter_fields( $fields, $order_id = 0 ) {
	if ( ! is_array( $fields ) ) {
		return $fields;
	}

	foreach ( bacs_account_label_map() as $key => $label ) {
		if ( isset( $fields[ $key ] ) && is_array( $fields[ $key ] ) ) {
			$fields[ $key ]['label'] = $label;
		}
	}

	return $fields;
}
// اولویت ۹۹ تا بعد از افزونه‌های فارسی‌ساز ووکامرس اجرا شود.
add_filter( 'woocommerce_bacs_account_fields', 'bacs_account_label_filter_fields', 99, 2 );
