<?php
/**
 * Plugin Name:       BACS Account Label (fa)
 * Plugin URI:        https://github.com/pedramhimself/Claude
 * Description:       اصلاح برچسب «شماره کارت» به «شماره حساب» و افزودن یادداشت کوتاه واریز از طریق اپلیکیشن پل، در صفحه ثبت سفارش و ایمیل‌های سفارش (درگاه انتقال بانکی مستقیم ووکامرس).
 * Version:           1.1.0
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
 * متن یادداشت واریز.
 *
 * عبارت {total} با مبلغ نهایی سفارش جایگزین می‌شود.
 * برای حذف کامل یادداشت، مقدار را به رشته خالی ('') تغییر دهید.
 */
function bacs_account_label_notice_text() {
	return apply_filters(
		'bacs_account_label_notice_text',
		'لطفاً مبلغ سفارش را دقیقاً با همین خُرده ({total}) از طریق اپلیکیشن «پل» واریز کنید.'
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

/**
 * متن یادداشت را با مبلغ سفارش می‌سازد.
 *
 * @param WC_Order $order      سفارش.
 * @param bool     $plain_text خروجی متنی ساده (برای ایمیل متنی).
 * @return string رشته خالی اگر یادداشتی تنظیم نشده باشد.
 */
function bacs_account_label_build_notice( $order, $plain_text = false ) {
	$text = trim( (string) bacs_account_label_notice_text() );

	if ( '' === $text || ! $order instanceof WC_Order ) {
		return '';
	}

	$total = $order->get_formatted_order_total();

	if ( $plain_text ) {
		$total = wp_strip_all_tags( $total );
	}

	return str_replace( '{total}', $total, $text );
}

/**
 * یادداشت واریز را بالای مشخصات بانکی در صفحه «سفارش دریافت شد» چاپ می‌کند.
 *
 * @param int $order_id شناسه سفارش.
 */
function bacs_account_label_thankyou_notice( $order_id ) {
	$notice = bacs_account_label_build_notice( wc_get_order( $order_id ) );

	if ( '' === $notice ) {
		return;
	}

	printf(
		'<p class="bacs-account-label__notice" style="margin:0 0 1em;padding:.9em 1em;border-radius:6px;background:#fff8e5;border-inline-start:4px solid #f0b849;font-weight:600;">%s</p>',
		wp_kses_post( $notice )
	);
}
// اولویت ۹ تا پیش از متن دستورالعمل و مشخصات بانکی درگاه (اولویت ۱۰) چاپ شود.
add_action( 'woocommerce_thankyou_bacs', 'bacs_account_label_thankyou_notice', 9 );

/**
 * یادداشت واریز را بالای مشخصات بانکی در ایمیل سفارش چاپ می‌کند.
 *
 * شرط‌ها با WC_Gateway_BACS::email_instructions یکسان است تا یادداشت فقط در
 * همان ایمیلی بیاید که مشخصات بانکی در آن چاپ می‌شود.
 *
 * @param WC_Order $order         سفارش.
 * @param bool     $sent_to_admin ایمیل مدیر.
 * @param bool     $plain_text    ایمیل متنی ساده.
 */
function bacs_account_label_email_notice( $order, $sent_to_admin = false, $plain_text = false ) {
	if ( $sent_to_admin || ! $order instanceof WC_Order ) {
		return;
	}

	if ( 'bacs' !== $order->get_payment_method() || ! $order->has_status( 'on-hold' ) ) {
		return;
	}

	$notice = bacs_account_label_build_notice( $order, $plain_text );

	if ( '' === $notice ) {
		return;
	}

	if ( $plain_text ) {
		echo esc_html( $notice ) . PHP_EOL . PHP_EOL;
		return;
	}

	printf(
		'<p style="margin:0 0 16px;padding:12px 14px;border-radius:6px;background:#fff8e5;border-right:4px solid #f0b849;font-weight:bold;">%s</p>',
		wp_kses_post( $notice )
	);
}
// اولویت ۹ تا پیش از WC_Gateway_BACS::email_instructions (اولویت ۱۰) چاپ شود.
add_action( 'woocommerce_email_before_order_table', 'bacs_account_label_email_notice', 9, 3 );
