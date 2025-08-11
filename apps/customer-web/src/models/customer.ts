// Customer and order submission models
export interface CustomerInfo {
    name: string;
    email: string;
    phone: string;
}

export interface DeliveryInfo {
    address: string;
    city: string;
    zipCode: string;
    instructions?: string;
}

export interface PaymentInfo {
    cardNumber: string;
    expiryDate: string;
    cvv: string;
    nameOnCard: string;
}