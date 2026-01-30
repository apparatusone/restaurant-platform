import { apiClient } from '../client.js';
import type { Payment } from '../../types/restaurant.js';

class PaymentsAPI {
    async processPayment(data: {
        check_id: number;
        amount: number;
        payment_type: 'cash' | 'credit_card' | 'debit_card';
        card_number?: string;
    }): Promise<Payment> {
        return apiClient.request<Payment>(`payments`, {
            method: 'POST',
            body: JSON.stringify({
                check_id: data.check_id,
                amount: data.amount,
                payment_type: data.payment_type,
                card_number: data.card_number
            })
        });
    }

    async getPaymentsForCheck(checkId: number): Promise<Payment[]> {
        return apiClient.request<Payment[]>(`payments/check/${checkId}`);
    }

    async getCheckPaymentSummary(checkId: number): Promise<any> {
        return apiClient.request<any>(`payments/check/${checkId}/summary`);
    }
}

export const paymentsApi = new PaymentsAPI();
